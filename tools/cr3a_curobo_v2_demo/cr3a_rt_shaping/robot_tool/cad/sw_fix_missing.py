# [2026-10-16] 실행 중인 SolidWorks 의 활성 어셈블리에서, 표시용 삼각형이 없는 면이 있는 몸체를 찾아 SolidWorks 에 삼각형을 새로
# 만들게 해서 (IBody2.GetTessellation → ITessellation.Tessellate, 화면 표시와 상관없음) 다시 읽는다. 읽기만 (저장·수정 없음).
#   py -3.11 sw_fix_missing.py cad_raw.npz cad_fix.npz
# 처음 추출 (sw_extract.py) 은 Face2.GetTessTriangles (화면에 그린 삼각형) 만 읽어서, 그리퍼 몸통 A.STEP 처럼 화면 삼각형이 없는
# 면이 빠졌다 (A.STEP 삼각형이 14 mm 조각뿐, 부품 상자는 Link6 기준 z 12~144 mm).
import json
import sys
import time
import types

import numpy as np
import win32com.client as w32
from win32com.client import dynamic


def get(obj, name, *args):
    v = getattr(obj, name)
    if isinstance(v, types.MethodType):
        return v(*args)
    return v(*args) if args else v


def tess_body(b):
    """몸체 하나 → 삼각형 [N, 9] (부품 좌표, m). ITessellation 으로 새로 만듦."""
    t = get(b, 'GetTessellation', None)
    t.NeedFaceFacetMap = True
    t.NeedVertexNormal = False
    t.NeedVertexParams = False
    t.ImprovedQuality = True
    t.MatchType = 1                       # swTesselationMatchFacetTopology (이웃 면과 꼭짓점 공유)
    ok = get(t, 'Tessellate')
    if not ok:
        raise RuntimeError('Tessellate 실패')
    nv = get(t, 'GetVertexCount')
    V = np.array([get(t, 'GetVertexPoint', i) for i in range(nv)], float)
    nf = get(t, 'GetFacetCount')
    F = []
    for f in range(nf):
        fins = get(t, 'GetFacetFins', f)
        F.append([get(t, 'GetFinVertices', fn)[0] for fn in fins])
    F = np.array(F, int)
    return V[F].reshape(-1, 9)


def main(src, dst):
    d = np.load(src)
    meta = json.loads(str(d['meta']))
    arrays = {k: d[k] for k in d.files if k != 'meta'}
    sw = dynamic.Dispatch(w32.GetActiveObject('SldWorks.Application')._oleobj_)
    model = sw.ActiveDoc
    print('active:', get(model, 'GetTitle'))
    comps = model.GetComponents(False)
    byname = {c.Name2: c for c in comps}
    t0 = time.time()
    fixed = []
    for m in meta:
        if m['n_children']:
            continue
        c = byname.get(m['name'])
        if c is None:
            print('  ! 부품 없음:', m['name'])
            continue
        bodies = c.GetBodies2(-1) or []
        miss = tot = 0
        for b in bodies:
            for f in (get(b, 'GetFaces') or []):
                tot += 1
                if not get(f, 'GetTessTriangles', True):
                    miss += 1
        m['faces'] = tot
        m['faces_no_tess'] = miss
        if miss == 0:
            continue
        tris = [tess_body(b) for b in bodies]
        T = np.concatenate(tris) if tris else np.zeros((0, 9))
        old = int(m.get('n_tri', 0))
        arrays[f"tri{m['i']}"] = T.astype(np.float32)
        m['n_tri'] = int(len(T))
        m['retess'] = True
        P = T.reshape(-1, 3)
        m['raw_min'], m['raw_max'] = P.min(0).tolist(), P.max(0).tolist()
        fixed.append((m['name'], tot, miss, old, len(T)))
        print(f"  고침 {m['name'][-60:]}: 면 {tot} 중 표시 삼각형 없음 {miss} → 삼각형 {old} → {len(T)}  ({time.time() - t0:.0f} s)", flush=True)
    np.savez_compressed(dst, meta=json.dumps(meta, ensure_ascii=False), **arrays)
    print(f'저장 {dst}: 고친 부품 {len(fixed)} 개, 면 확인 {sum(1 for m in meta if "faces" in m)} 부품, {time.time() - t0:.0f} s')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
