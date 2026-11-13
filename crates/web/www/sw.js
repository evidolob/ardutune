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
      return cached ?? fresh;
    }),
  );
});
