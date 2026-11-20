#!/usr/bin/env python3
"""관절 그래프 탭 — 관절 6 개를 오실로스코프처럼 최근 10 초 (미로봇 GUI scope_panel 과 같은 자리).
선: 성형 전 명령 (회색 점선) · 성형 후 명령 = WMX 로 보낸 것 (파랑) · 실제 위치 = 엔코더 (주황).
[▶ 그래프 켜기] 를 눌러야 쌓기 시작 (켤 때마다 비움), 변위 / 속도 / 가속도를 고름. 보이는 탭일 때만 그림.
[2026-10-27 R-1 ④] 단추를 실제로 연결 (전엔 안 이어져 늘 쌓고 그렸음): 켜짐 = running (MainWindow 가 이때만 쌓고 그림), 끄면 마지막 그림 그대로."""
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

SERIES = (('u', '성형 전 명령', '#8c959f', QtCore.Qt.DashLine),
          ('y', '성형 후 명령 (WMX 로 보냄)', '#0969da', QtCore.Qt.SolidLine),
          ('fb', '실제 위치 (엔코더)', '#e36209', QtCore.Qt.SolidLine))
DRAW = (('u', 1.4), ('fb', 3.0), ('y', 1.4))       # 그리는 순서·굵기: 실제 (굵게) 위에 성형 후 명령 (가늘게) — 겹쳐도 둘 다 보임
KINDS = (('변위', '°'), ('속도', '°/s'), ('가속도', '°/s²'))


class ScopePlot(QtWidgets.QWidget):
    def __init__(self, panel):
        super().__init__()
        self.panel = panel
        self.setMinimumSize(420, 300)

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#ffffff'))
        d = self.panel.data
        if d is None:
            p.setPen(QtGui.QColor('#57606a'))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter, '[▶ 그래프 켜기] 를 누르면 최근 10 초를 그립니다')
            return
        t = d['t']
        kind = self.panel.cb_kind.currentIndex()
        unit = KINDS[kind][1]
        on = {k: c.isChecked() for k, c in self.panel.chk.items()}
        lanes = 6
        top, left, right, bottom = 6, 64, 10, 22
        h = (self.height() - top - bottom) / lanes
        W = self.width() - left - right
        f = p.font()
        f.setPointSizeF(f.pointSizeF() * 0.9)
        p.setFont(f)
        for j in range(lanes):
            y0 = top + j * h
            r = QtCore.QRectF(left, y0 + 3, W, h - 6)
            p.setPen(QtGui.QPen(QtGui.QColor('#d0d7de'), 1))
            p.drawRect(r)
            ys = [self._series(d[k][:, j], t, kind) for k, *_ in SERIES if on[k]]
            if not ys:
                continue
            lo, hi = min(float(y.min()) for y in ys), max(float(y.max()) for y in ys)
            pad = max((hi - lo) * 0.12, {0: 0.5, 1: 2.0, 2: 10.0}[kind])
            lo, hi = lo - pad, hi + pad
            p.setPen(QtGui.QColor('#24292f'))
            p.drawText(QtCore.QRectF(2, y0 + 2, left - 8, 16), QtCore.Qt.AlignRight, f'J{j + 1}')
            p.setPen(QtGui.QColor('#57606a'))
            p.drawText(QtCore.QRectF(2, y0 + 17, left - 8, 14), QtCore.Qt.AlignRight, f'{hi:.0f}')
            p.drawText(QtCore.QRectF(2, y0 + h - 18, left - 8, 14), QtCore.Qt.AlignRight, f'{lo:.0f} {unit}')
            look = {k: (color, style) for k, _, color, style in SERIES}
            for k, width in DRAW:
                if not on[k]:
                    continue
                color, style = look[k]
                y = self._series(d[k][:, j], t, kind)
                step = max(1, len(t) // int(W))
                xs = left + W * (t[::step] - t[0]) / (t[-1] - t[0])
                yy = r.bottom() - r.height() * (y[::step] - lo) / (hi - lo)
                path = QtGui.QPainterPath(QtCore.QPointF(xs[0], yy[0]))
                for a, b in zip(xs[1:], yy[1:]):
                    path.lineTo(a, b)
                p.setPen(QtGui.QPen(QtGui.QColor(color), width, style))
                p.drawPath(path)
        p.setPen(QtGui.QColor('#57606a'))
        for s in range(0, 11, 2):
            x = left + W * s / 10
            p.drawText(QtCore.QRectF(x - 24, self.height() - bottom + 4, 48, 16), QtCore.Qt.AlignHCenter,
                       f'{s - 10} s' if s < 10 else '지금')

    @staticmethod
    def _series(x, t, kind):
        for _ in range(kind):
            x = np.gradient(x, t)
        return x


class ScopePanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.data = None
        v = QtWidgets.QVBoxLayout(self)
        v.setContentsMargins(4, 4, 4, 4)
        row = QtWidgets.QHBoxLayout()
        self.btn_run = QtWidgets.QPushButton('▶ 그래프 켜기', checkable=True)
        self.btn_run.toggled.connect(self._on_run)
        row.addWidget(self.btn_run)
        self.cb_kind = QtWidgets.QComboBox()
        self.cb_kind.addItems([k for k, _ in KINDS])
        self.cb_kind.currentIndexChanged.connect(lambda _: self.plot.update())
        row.addWidget(self.cb_kind)
        row.addSpacing(10)
        self.chk = {}
        for k, name, color, _ in SERIES:
            c = QtWidgets.QCheckBox(name, checked=True)
            c.setStyleSheet(f'QCheckBox{{color:{color};font-weight:bold;}}')
            c.toggled.connect(lambda _: self.plot.update())
            self.chk[k] = c
            row.addWidget(c)
        row.addStretch(1)
        v.addLayout(row)
        self.plot = ScopePlot(self)
        v.addWidget(self.plot, 1)

    @property
    def running(self):
        return self.btn_run.isChecked()

    def _on_run(self, on):
        """켜기 = 그림을 비우고 새로 쌓기 시작 (쌓는 것은 MainWindow — 같은 단추의 toggled 로 버퍼를 비움), 끄기 = 마지막 그림 그대로."""
        self.btn_run.setText('■ 그래프 끄기' if on else '▶ 그래프 켜기')
        if on:
            self.set_data(None)

    def set_data(self, d):
        self.data = d
        self.plot.update()
