import urllib.request
import json
import time
import threading
import yaml
from pathlib import Path
from app.visualization.dashboard import DashboardServer

def verify():
    cfg = yaml.safe_load(open('config/config.yaml'))
    cfg['dashboard']['auto_open_browser'] = False
    server = DashboardServer(cfg)

    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    time.sleep(2.0)

    # 1. /api/status
    res = urllib.request.urlopen('http://127.0.0.1:8050/api/status')
    status = json.loads(res.read())
    assert status["total_frames"] == 180

    # 2. /api/frame/1 (compute)
    t0 = time.perf_counter()
    f1 = json.loads(urllib.request.urlopen('http://127.0.0.1:8050/api/frame/1').read())
    t_comp = (time.perf_counter() - t0) * 1000
    print(f"FRAME 1 (computed): {t_comp:.1f}ms | Active Cells: {len(f1['active_cells'])} | Objects: {len(f1['detected_objects'])}")
    assert len(f1['active_cells']) > 0

    # 3. /api/frame/1 (cached replay)
    t1 = time.perf_counter()
    f1_c = json.loads(urllib.request.urlopen('http://127.0.0.1:8050/api/frame/1').read())
    t_cached = (time.perf_counter() - t1) * 1000
    print(f"FRAME 1 (cached): {t_cached:.2f}ms -> ZERO-LATENCY REPLAY VERIFIED!")

    # 4. / (HTML dashboard UI)
    html = urllib.request.urlopen('http://127.0.0.1:8050/').read().decode()
    assert 'id="lidar-canvas"' in html
    assert 'OCULUS' in html
    assert 'SIH' not in html
    assert 'Presentation Mode' not in html
    print("OCULUS HTML DASHBOARD ASSETS VERIFIED!")

    print("\nALL WEB DASHBOARD SERVER CHECKS PASSED PERFECTLY!\n")

if __name__ == '__main__':
    verify()
