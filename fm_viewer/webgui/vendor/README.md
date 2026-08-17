# Vendored JavaScript

Five third-party files, listed with their exact upstream source and hash in
[`vendor.json`](vendor.json). Each was verified byte-identical to the published
artifact; nothing here is patched.

| File | Package | Version |
|---|---|---|
| `three.min.js` | three | 0.147.0 (r147) |
| `ColladaLoader.js` | three | 0.147.0 (r147) |
| `STLLoader.js` | three | 0.147.0 (r147) |
| `OrbitControls.js` | three | 0.147.0 (r147) |
| `URDFLoader.js` | urdf-loader | 0.12.1 |

## Why they are vendored

The viewer is a dependency-free page. An operator opens it straight off the
filesystem — `open .../webgui/index.html?ws=ws://<rig>:8765` — with no build step, no
npm, and no network beyond the rig's own websocket. Both recorder install scripts
hand out that command. A CDN reference would break the one property that makes this
the operator surface for a machine that has nothing installed on it.

## Why r147 specifically

r147 is the **last** three.js release that ships `examples/js` — the plain-`<script>`
builds of the loaders and controls. r148 removed that directory; from r149 on the
examples are ES modules only. So the pin is a ceiling, not neglect.

Moving past r147 is a port, not a bump: the page would need ES modules and an import
map, which changes how it loads and what it can do from `file://`. That is worth
doing when the viewer is next worked on properly — it is not a dependency update.

For the record at the time of writing, no published advisory affects these versions.
The two known three.js advisories (GHSA-7vvq-7r29-5vg3, GHSA-fq6p-x6j3-cmmq) are
fixed in 0.137.0 and 0.125.0, both below the pin, and urdf-loader has none.

## Changing a file here

Update the entry in `vendor.json` — version, url, and sha256 together.
`test/test_vendor.py` compares the manifest against the files on disk and fails when
they drift, so a hand-edited library or a silent replacement is a red build. That is
the point: these are the files no one reviews, and a hash is the only thing that
notices.

Recompute a hash with:

```bash
shasum -a 256 three.min.js
```
