/**
 * 极简静态服务器（只给预览器用）。
 * 用法：node serve.mjs  [--port 8712] [--no-open]
 */
import http from 'node:http';
import { promises as fs } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawn } from 'node:child_process';

const ROOT = path.dirname(fileURLToPath(import.meta.url));
const ARGS = process.argv.slice(2);
const argOf = (name, fallback) => {
  const i = ARGS.indexOf(name);
  return i >= 0 && ARGS[i + 1] ? ARGS[i + 1] : fallback;
};
const START_PORT = Number(argOf('--port', process.env.PORT || 8712));
const OPEN_BROWSER = !ARGS.includes('--no-open');

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.urdf': 'text/xml; charset=utf-8',
  '.xml': 'text/xml; charset=utf-8',
  '.stl': 'model/stl',
  '.dae': 'model/vnd.collada+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.svg': 'image/svg+xml',
  '.ico': 'image/x-icon',
};

const server = http.createServer(async (req, res) => {
  try {
    const url = new URL(req.url, 'http://127.0.0.1');
    let rel = decodeURIComponent(url.pathname);
    if (rel.endsWith('/')) rel += 'index.html';
    const file = path.join(ROOT, path.normalize(rel).replace(/^([/\\])+/, ''));
    if (!file.startsWith(ROOT)) {
      res.writeHead(403).end('403 forbidden');
      return;
    }
    const data = await fs.readFile(file);
    res.writeHead(200, {
      'Content-Type': MIME[path.extname(file).toLowerCase()] || 'application/octet-stream',
      'Content-Length': data.length,
      'Cache-Control': 'no-cache',
    });
    res.end(data);
  } catch {
    res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' }).end(`404 ${req.url}`);
  }
});

function listen(port, tries = 12) {
  server.once('error', (err) => {
    if (err.code === 'EADDRINUSE' && tries > 0) listen(port + 1, tries - 1);
    else {
      console.error(err);
      process.exit(1);
    }
  });
  server.listen(port, '127.0.0.1', () => {
    const url = `http://127.0.0.1:${port}/`;
    console.log(`\n  Miku Arm 预览器已启动 →  ${url}\n  关闭这个窗口即可停止服务。\n`);
    if (OPEN_BROWSER) openUrl(url);
  });
}

function openUrl(url) {
  const cmd = process.platform === 'win32' ? 'cmd' : process.platform === 'darwin' ? 'open' : 'xdg-open';
  const args = process.platform === 'win32' ? ['/c', 'start', '', url] : [url];
  try {
    spawn(cmd, args, { detached: true, stdio: 'ignore' }).unref();
  } catch {
    /* 打不开就算了，用户自己点链接 */
  }
}

listen(START_PORT);
