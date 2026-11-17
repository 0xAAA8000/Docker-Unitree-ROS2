"""スマホ用の HTTP サーバー部分 (ROS に依存しない).

GET  /              地図画面 (web/index.html)
GET  /api/state     現在地, 経路, ゴールの状態などの JSON
GET  /api/map.png   2D 地図の画像. 大きさと原点は X-Map-* ヘッダで返す
POST /api/goal      {"x": .., "y": .., "yaw": rad または null} を map 座標で
POST /api/cancel    走行中のゴールを取り消す

token を設定すると /api/* は ?key=<token> が無いと 403 を返す.
backend は get_state(), get_map() -> (meta dict, png bytes) | None,
send_goal(x, y, yaw) -> (ok, message), cancel() -> (ok, message) を持つもの.
"""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from urllib.parse import parse_qs, urlparse
import zlib

import numpy as np

# 地図の色 (グレースケール)
GRAY_FREE = 255
GRAY_UNKNOWN = 190
GRAY_OCCUPIED = 30


def occupancy_to_gray(data, width, height):
    """OccupancyGrid の data (行 0 が y の小さい側) を, 上が y の大きい側の画像にする."""
    occ = np.frombuffer(bytes(data), np.int8, count=width * height).reshape(height, width)
    img = np.full(occ.shape, GRAY_UNKNOWN, np.uint8)
    img[(occ >= 0) & (occ < 50)] = GRAY_FREE
    img[occ >= 50] = GRAY_OCCUPIED
    return np.ascontiguousarray(img[::-1])


def known_bounds(data, width, height, resolution, origin_x, origin_y):
    """未知でないセルを囲む範囲 [x0, y0, x1, y1] (map 座標). 全部未知なら None."""
    occ = np.frombuffer(bytes(data), np.int8, count=width * height).reshape(height, width)
    rows = np.flatnonzero((occ >= 0).any(axis=1))
    cols = np.flatnonzero((occ >= 0).any(axis=0))
    if not len(rows):
        return None
    return [float(origin_x + cols[0] * resolution), float(origin_y + rows[0] * resolution),
            float(origin_x + (cols[-1] + 1) * resolution),
            float(origin_y + (rows[-1] + 1) * resolution)]


def encode_png_gray(img):
    """(高さ, 幅) uint8 配列を 8bit グレースケール PNG にする."""
    h, w = img.shape
    raw = np.zeros((h, w + 1), np.uint8)   # 各行の先頭はフィルタ種別 (0 = なし)
    raw[:, 1:] = img

    def chunk(kind, body):
        c = kind + body
        return len(body).to_bytes(4, 'big') + c + zlib.crc32(c).to_bytes(4, 'big')

    ihdr = w.to_bytes(4, 'big') + h.to_bytes(4, 'big') + bytes([8, 0, 0, 0, 0])
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr)
            + chunk(b'IDAT', zlib.compress(raw.tobytes(), 6)) + chunk(b'IEND', b''))


def make_server(host, port, backend, index_html, token=''):
    """ThreadingHTTPServer を作る (serve_forever() は呼び出し側で)."""

    class Handler(BaseHTTPRequestHandler):
        server_version = 'robot_goal_web'

        def log_message(self, fmt, *args):   # 1 秒ごとの問い合わせでログが埋まるので出さない
            pass

        def _send(self, code, body, ctype, headers=None):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (headers or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != 'HEAD':
                self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode(),
                       'application/json; charset=utf-8')

        def _authorized(self, url):
            if not token:
                return True
            key = parse_qs(url.query).get('key', [''])[0]
            return hmac.compare_digest(key.encode(), token.encode())

        def do_GET(self):
            url = urlparse(self.path)
            if url.path in ('/', '/index.html'):
                self._send(200, index_html, 'text/html; charset=utf-8')
                return
            if not url.path.startswith('/api/'):
                self._json(404, {'error': 'not found'})
                return
            if not self._authorized(url):
                self._json(403, {'error': 'key が違う'})
                return
            if url.path == '/api/state':
                self._json(200, backend.get_state())
            elif url.path == '/api/map.png':
                m = backend.get_map()
                if m is None:
                    self._json(404, {'error': '地図がまだ届いていない'})
                    return
                meta, png = m
                self._send(200, png, 'image/png', {
                    'X-Map-Version': str(meta['version']),
                    'X-Map-Width': str(meta['width']),
                    'X-Map-Height': str(meta['height']),
                    'X-Map-Resolution': repr(meta['resolution']),
                    'X-Map-Origin-X': repr(meta['origin_x']),
                    'X-Map-Origin-Y': repr(meta['origin_y']),
                    'X-Map-Known': ','.join(repr(v) for v in meta['known'] or []),
                })
            else:
                self._json(404, {'error': 'not found'})

        def do_POST(self):
            url = urlparse(self.path)
            if not self._authorized(url):
                self._json(403, {'error': 'key が違う'})
                return
            try:
                n = int(self.headers.get('Content-Length', '0'))
                body = json.loads(self.rfile.read(n) or b'{}') if n <= 10000 else None
            except (ValueError, json.JSONDecodeError):
                body = None
            if not isinstance(body, dict):
                self._json(400, {'ok': False, 'message': 'JSON が読めない'})
                return
            if url.path == '/api/goal':
                try:
                    x, y = float(body['x']), float(body['y'])
                    yaw = body.get('yaw')
                    yaw = None if yaw is None else float(yaw)
                except (KeyError, TypeError, ValueError):
                    self._json(400, {'ok': False, 'message': 'x, y (と yaw) を数値で送る'})
                    return
                if not all(math.isfinite(v) for v in (x, y, 0.0 if yaw is None else yaw)):
                    self._json(400, {'ok': False, 'message': '数値が不正'})
                    return
                ok, msg = backend.send_goal(x, y, yaw)
                self._json(200 if ok else 503, {'ok': ok, 'message': msg})
            elif url.path == '/api/cancel':
                ok, msg = backend.cancel()
                self._json(200, {'ok': ok, 'message': msg})
            else:
                self._json(404, {'ok': False, 'message': 'not found'})

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
