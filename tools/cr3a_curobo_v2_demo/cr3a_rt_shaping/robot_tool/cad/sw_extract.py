# SolidWorks (실행 중, 활성 어셈블리) 에서 모든 끝 부품의 변환 행렬·상자·표시용 삼각형을 읽어 npz 로 (읽기만, 저장·수정 없음)
#   py -3.11 sw_extract.py cad_raw.npz
# 늦은 바인딩: 형 정보가 있는 객체는 메서드, 없는 객체는 속성처럼 읽혀서 둘 다 받아 준다.
# [2026-10-16 고침] 몸체는 고체 + 면 (GetBodies2(-1)) 모두 — 그리퍼 몸통 A.STEP 은 STEP 가져오기에서 면 몸체 50 여 개로 들어와,
#   고체만 (GetBodies2(0)) 읽었을 때 몸통이 통째로 빠졌음 (14 mm 조각만 남음). J6 의 면 몸체도 이제 같이 읽음 (전에는 sw_j6.py 로 따로).
#   표시 삼각형이 없는 면 (넓이 0 인 찌꺼기 면, J6 면 몸체의 85 cm² 면 하나) 은 건너뛰고 수·넓이를 기록.
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


out = sys.argv[1]
sw = dynamic.Dispatch(w32.GetActiveObject('SldWorks.Application')._oleobj_)
model = sw.ActiveDoc
comps = model.GetComponents(False)
meta, arrays = [], {}
t0 = time.time()
for i, c in enumerate(comps):
    ch = get(c, 'GetChildren')
    nch = 0 if ch is None else len(ch)
    par = get(c, 'GetParent')
    xf = c.Transform2
    rec = {'i': i, 'name': c.Name2, 'path': get(c, 'GetPathName'), 'parent': par.Name2 if par is not None else None,
           'n_children': nch, 'xf': list(xf.ArrayData) if xf is not None else None,
           'box': list(c.GetBox(False, False)), 'supp': bool(get(c, 'IsSuppressed')), 'visible': int(c.Visible)}
    if nch == 0:
        bodies = c.GetBodies2(-1) or []            # 고체 + 면
        n_solid = len(c.GetBodies2(0) or [])
        tris, miss, miss_area = [], 0, 0.0
        for b in bodies:
            for f in (get(b, 'GetFaces') or []):
                t = get(f, 'GetTessTriangles', True)
                if t:
                    tris.append(np.asarray(t, dtype=np.float64).reshape(-1, 9))
                else:
                    miss += 1
                    miss_area += float(get(f, 'GetArea'))
        T = np.concatenate(tris) if tris else np.zeros((0, 9))
        arrays[f'tri{i}'] = T.astype(np.float32)
        rec.update(n_bodies=len(bodies), n_solid=n_solid, n_tri=int(len(T)), faces_no_tess=miss, area_no_tess_cm2=round(miss_area * 1e4, 2))
        if len(T):
            P = T.reshape(-1, 3)
            rec['raw_min'] = P.min(0).tolist()
            rec['raw_max'] = P.max(0).tolist()
    meta.append(rec)
    print(f"{i:2d} {rec['name'][-58:]:58s} ch={nch} 몸체={rec.get('n_bodies', '-')} (고체 {rec.get('n_solid', '-')}) "
          f"tri={rec.get('n_tri', '-')} 빠진 면 {rec.get('faces_no_tess', '-')} ({rec.get('area_no_tess_cm2', '-')} cm²)  {time.time() - t0:.1f}s", flush=True)
np.savez_compressed(out, meta=json.dumps(meta, ensure_ascii=False), **arrays)
print('saved', out)
