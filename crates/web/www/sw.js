// Offline use: every file of the app is cached on first load; later loads
// come from the cache and refresh it in the background. The build script
// stamps VERSION so a new release replaces the old cache.
const VERSION = "dev";
const CACHE = `ardutune-${VERSION}`;
const FILES = [
  "./",
  "index.html",
  "app.js",
  "style.css",
  "web-tauri.js",
  "worker.js",
  "pkg/ardutune_web.js",
  "pkg/ardutune_web_bg.wasm",
  "manifest.webmanifest",
  "icons/128x128.png",
  "icons/128x128@2x.png",
  "icons/icon.png",
];

self.addEventListener("install", (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(FILES)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", (e) => {
  e.waitUntil(
    caches
      .keys()
      .then((keys) => Promise.all(keys.filter((k) => k.startsWith("ardutune-") && k !== CACHE).map((k) => caches.delete(k))))
      .then(() => self.clients.claim()),
  );
});

// Cross-origin isolation (SharedArrayBuffer for the serial link): GitHub
// Pages can't set these headers, so every response gets them here.
function isolated(response) {
  if (!response || response.status === 0) return response;
  const headers = new Headers(response.headers);
  headers.set("Cross-Origin-Opener-Policy", "same-origin");
  headers.set("Cross-Origin-Embedder-Policy", "require-corp");
  headers.set("Cross-Origin-Resource-Policy", "same-origin");
  return new Response(response.body, { status: response.status, statusText: response.statusText, headers });
}

self.addEventListener("fetch", (e) => {
  if (e.request.method !== "GET" || new URL(e.request.url).origin !== location.origin) return;
  e.respondWith(
    caches.open(CACHE).then(async (cache) => {
      const cached = await cache.match(e.request, { ignoreSearch: true });
      const fresh = fetch(e.request)
        .then((r) => {
          if (r.ok) cache.put(e.request, r.clone());
          return r;
        })
        .catch(() => cached);
      return isolated(cached ?? (await fresh));
    }),
  );
});
