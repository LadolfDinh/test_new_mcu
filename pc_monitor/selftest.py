# Tu kiem tra giao dien voi gia lap ESP32 (khong hien cua so), luu anh logs/test_giaodien.png. Chay: python selftest.py
import os, sys, threading
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, D)
os.chdir(D)

import sim_esp32
threading.Thread(target=sim_esp32.main, daemon=True).start()

from pyqtgraph.Qt import QtWidgets, QtCore
import app

qa = QtWidgets.QApplication(sys.argv)
link = app.UdpLink()
link.start()
w = app.MainWindow(link, 10)
w.resize(1200, 760)
w.setAttribute(QtCore.Qt.WidgetAttribute.WA_DontShowOnScreen, True)
w.show()

def done():
    os.makedirs(os.path.join(D, "logs"), exist_ok=True)
    w.grab().save(os.path.join(D, "logs", "test_giaodien.png"))
    print("esp_ip:", link.esp_ip, "| khung:", link.n_frames, "| mat:", link.n_lost,
          "| loi CRC:", link.n_bad, "| RTT ms:", link.rtt_ms, "| toc do ve:", round(w.rate), "khung/s")
    print("status:", w.lbl_status.text().replace("\n", " || "))
    qa.quit()

QtCore.QTimer.singleShot(6000, done)
qa.exec()
link.stop()
