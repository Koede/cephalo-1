"""GLB -> compact per-colour meshes (int16 positions at 0.1 mm, uint32 indices), decimated, base64 JSON for the viewer."""
import sys, json, base64, numpy as np, trimesh, fast_simplification as FS
out = {}
for tag in ("stand", "walk", "fly"):
    sc = trimesh.load(f"chilo_c2_{tag}.glb"); groups = {}
    for g in sc.dump():
        fc = g.visual.face_colors[:, :3]
        for c in np.unique(fc, axis=0):
            m = np.all(fc == c, axis=1); groups.setdefault(tuple(int(x) for x in c), []).append(trimesh.Trimesh(g.vertices, g.faces[m], process=False))
    lst = []; nf = 0
    for c, ms in groups.items():
        t = trimesh.util.concatenate(ms); t.merge_vertices(); t.remove_unreferenced_vertices()
        V, F = np.asarray(t.vertices, np.float32), np.asarray(t.faces, np.int64)
        if len(F) > 8000:
            V, F = FS.simplify(V, F.astype(np.int32), target_reduction=0.78)
        q = np.round(V * 10000).astype(np.int16)                         # 0.1 mm
        lst.append(dict(c=c, v=base64.b64encode(q.tobytes()).decode(), f=base64.b64encode(np.asarray(F, np.uint32).tobytes()).decode()))
        nf += len(F)
    s = json.dumps(lst); open(f"viewer/model_{tag}.js", "w").write("window.MODEL=window.MODEL||{};window.MODEL['" + tag + "']=" + s + ";")
    print(tag, len(lst), "groups", nf, "faces", round(len(s) / 1e6, 1), "MB")
