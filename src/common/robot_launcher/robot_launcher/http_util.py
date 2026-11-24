"""小さな JSON API 用の HTTP サーバー (ROS に依存しない).

routes は {('GET', '/api/state'): handler, ...}. handler(query: dict, body: dict) は
(HTTP ステータス, JSON にできる値) を返す. pages は {'/': HTML のバイト列}.
token を設定すると /api/* は ?key=<token> が無いと 403 を返す (画面そのものは返す).
"""
import hmac
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from urllib.parse import parse_qs, urlparse


def make_server(host, port, routes, pages, token=''):
    class Handler(BaseHTTPRequestHandler):
        server_version = 'robot_launcher'

        def log_message(self, fmt, *args):   # 定期的な問い合わせでログが埋まるので出さない
            pass

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code, obj):
            self._send(code, json.dumps(obj, ensure_ascii=False).encode(),
                       'application/json; charset=utf-8')

        def _handle(self, method):
            url = urlparse(self.path)
            query = {k: v[0] for k, v in parse_qs(url.query).items()}
            if method == 'GET' and url.path in pages:
                self._send(200, pages[url.path], 'text/html; charset=utf-8')
                return
            handler = routes.get((method, url.path))
            if handler is None:
                self._json(404, {'ok': False, 'message': 'not found'})
                return
            if token and not hmac.compare_digest(query.get('key', '').encode(), token.encode()):
                self._json(403, {'ok': False, 'message': 'URL の key が違う'})
                return
            body = {}
            if method == 'POST':
                try:
                    n = int(self.headers.get('Content-Length', '0'))
                    body = json.loads(self.rfile.read(n) or b'{}') if n <= 10000 else None
                except (ValueError, json.JSONDecodeError):
                    body = None
                if not isinstance(body, dict):
                    self._json(400, {'ok': False, 'message': 'JSON が読めない'})
                    return
            try:
                code, obj = handler(query, body)
            except Exception as e:   # noqa: BLE001 (サーバーを落とさず, 画面に理由を返す)
                code, obj = 500, {'ok': False, 'message': f'内部エラー: {e}'}
            self._json(code, obj)

        def do_GET(self):
            self._handle('GET')

        def do_POST(self):
            self._handle('POST')

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
