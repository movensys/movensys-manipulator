#!/usr/bin/env python3
"""입력성형 전역 패널 — 탭 밖. 목표 이동·반복·실시간 회피에 같은 성형기 하나 ([2026-10-27 사용자] 조그는 MoveIt Servo, 성형 없음).

성형기 정의 (방식·감쇠 없음·1 ms 칸·합치기) 는 shaper_def.py — 서버 (rt_server.py) 와 같은 코드.
[적용] = 멈춰 있을 때만 (움직이는 중에 성형기를 바꾸면 명령이 튐 — 작업기록 입력성형 1-8). 시스템이 켜져 있으면 서버가 받아들인 뒤에
  '적용 중' 으로 바뀜 (apply_requested → MainWindow → 서버 → mark_applied). 꺼져 있으면 바로 (다음 시스템 시작 때 이 값으로).
주파수 칸은 Enter·칸 나가기·화살표 때만 다시 계산 (치는 도중의 값 — 예: "15" 를 지우다 생긴 "1" — 으로 계산하지 않음).
"""
import math

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from shaper_def import METHODS, WARN_RESIDUAL, build, label, ms

METHOD_TIPS = {
    'Unshaped': '입력성형 없음 — 명령을 그대로 보냄 (지연 0)',
    'ZV': 'ZV: 임펄스 2 개 [½, ½], 길이 = 진동 주기의 반. 가장 짧지만 주파수가 0.97~1.03 배를 벗어나면 잔류 5 % 넘음',
    'ETM4': 'ETM4 (Kang 2019, m = 1): 임펄스 4 개 [1/6, 1/3, 1/3, 1/6], 길이 = 진동 주기. 0.93~1.11 배까지 잔류 5 % 이하',
    'ETM5': 'ETM5 (Kang 2019, m = 1): 임펄스 5 개 [1/8, ¼, ¼, ¼, 1/8], 길이 = 진동 주기. 0.94~1.07 배까지 잔류 5 % 이하',
}
GRID_TIP = ('주파수마다 정확한 값으로 성형기를 만들어 컨볼루션한 뒤, 최종 성형기의 임펄스를 가장 가까운 1 ms 칸 (WMX 통신 주기) 에\n'
            '놓습니다 (최대 0.5 ms 옮김). 같은 칸에 모인 임펄스는 하나로 합칩니다 (세기 합).\n'
            '지연 시간 = 성형기 길이 (첫 임펄스 ~ 마지막 임펄스): 모션이 끝나는 시각이 이만큼 늦어집니다 (출발은 그대로).')
BLUE, GREY, OK, WARN = '#0969da', '#8c959f', '#1a7f37', '#9a6700'


def nice_step(span, n=5):
    """눈금 간격: span 을 약 n 칸으로 나누는 1·2·2.5·5 × 10^k (길이가 얼마든)."""
    raw = max(span / n, 1e-9)
    mag = 10 ** math.floor(math.log10(raw))
    return next(m * mag for m in (1, 2, 2.5, 5, 10) if raw <= m * mag + 1e-12)


class ImpulsePlot(QtWidgets.QWidget):
    """임펄스 열 (세기 / 시각 ms, 1 ms 칸). 마지막 임펄스 시각 = 지연 시간."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(78)
        self.res = None

    def set_result(self, res):
        self.res = res
        self.update()

    def paintEvent(self, _):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        r = self.rect().adjusted(34, 6, -8, -18)
        p.fillRect(self.rect(), QtGui.QColor('#ffffff'))
        p.setPen(QtGui.QPen(QtGui.QColor('#d0d7de'), 1))
        p.drawRect(r)
        if self.res is None:
            return
        A, t = self.res['A'], self.res['t'] * 1000
        tmax = max(float(t[-1]) * 1.04, 10.0)
        amax = float(A.max()) * 1.12
        X = lambda v: r.left() + r.width() * v / tmax          # noqa: E731
        Y = lambda a: r.bottom() - r.height() * a / amax         # noqa: E731
        f = p.font()
        f.setPointSizeF(f.pointSizeF() * 0.85)
        p.setFont(f)
        p.setPen(QtGui.QColor('#57606a'))
        for v in np.arange(0, tmax, nice_step(tmax)):
            x = X(v)
            p.drawLine(QtCore.QPointF(x, r.bottom()), QtCore.QPointF(x, r.bottom() + 3))
            if x < r.right() - 30:                                   # 오른쪽 끝은 'ms' 자리
                p.drawText(QtCore.QRectF(x - 24, r.bottom() + 3, 48, 14), QtCore.Qt.AlignHCenter, f'{v:g}')
        p.drawText(QtCore.QRectF(r.right() - 40, r.bottom() + 3, 40, 14), QtCore.Qt.AlignRight, 'ms')
        p.drawText(QtCore.QRectF(0, r.top() - 2, 30, 14), QtCore.Qt.AlignRight, f'{amax / 1.12:.2f}')
        p.drawText(QtCore.QRectF(0, r.bottom() - 12, 30, 14), QtCore.Qt.AlignRight, '0')
        p.setPen(QtGui.QPen(QtGui.QColor(BLUE), 2.0))
        p.setBrush(QtGui.QColor(BLUE))
        for a, v in zip(A, t):
            p.drawLine(QtCore.QPointF(X(v), r.bottom()), QtCore.QPointF(X(v), Y(a)))
            p.drawEllipse(QtCore.QPointF(X(v), Y(a)), 2.4, 2.4)


class ShapingPanel(QtWidgets.QGroupBox):
    apply_requested = QtCore.pyqtSignal(object)     # (방식, ((켬, Hz) × 3)) — MainWindow 가 서버로 보냄. [R-1] 안 쓰던 changed 신호는 뺌

    def __init__(self, mono):
        super().__init__('입력성형 — 목표 이동 · 반복 · 실시간 회피에 적용 (조그는 성형 없음)')
        self.setStyleSheet('QGroupBox{font-weight:bold;} QGroupBox QLabel, QGroupBox QCheckBox, QGroupBox QPushButton,'
                           ' QGroupBox QDoubleSpinBox{font-weight:normal;}')
        self.setToolTip('감쇠 없는 시스템으로 가정 (ζ = 0) 해서 성형기를 만듭니다.\n' + GRID_TIP)
        self.applied = None
        self.pending = None                 # 서버에 보내고 답을 기다리는 설정
        v = QtWidgets.QVBoxLayout(self)
        v.setSpacing(3)
        v.setContentsMargins(8, 6, 8, 6)
        # 방식
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(0)
        row.addWidget(QtWidgets.QLabel('방식  '))
        self.grp = QtWidgets.QButtonGroup(self)
        for k, name in enumerate(METHODS):
            b = QtWidgets.QPushButton(name, checkable=True)
            b.setToolTip(METHOD_TIPS[name])
            b.setMinimumWidth(78)
            pos = 'first' if k == 0 else ('last' if k == len(METHODS) - 1 else 'mid')
            rad = {'first': 'border-top-left-radius:5px;border-bottom-left-radius:5px;',
                   'last': 'border-top-right-radius:5px;border-bottom-right-radius:5px;', 'mid': ''}[pos]
            b.setStyleSheet('QPushButton{border:1px solid #8c959f;background:#f6f8fa;padding:3px 8px;margin-left:-1px;'
                            f'border-radius:0px;{rad}}} QPushButton:checked{{background:{BLUE};color:white;'
                            f'border-color:{BLUE};font-weight:bold;}}')
            self.grp.addButton(b, k)
            row.addWidget(b)
        row.addStretch(1)
        v.addLayout(row)
        # 주파수 3 칸 + 미리보기
        body = QtWidgets.QHBoxLayout()
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(6)
        g.setVerticalSpacing(2)
        for c, text in enumerate(('', '주파수')):
            lb = QtWidgets.QLabel(text)
            lb.setStyleSheet(f'color:{GREY}')
            g.addWidget(lb, 0, c)
        self.rows = []
        for k in range(3):
            chk = QtWidgets.QCheckBox(f'{k + 1}')
            sp_f = QtWidgets.QDoubleSpinBox(decimals=2, singleStep=0.1, suffix=' Hz')
            sp_f.setRange(0.5, 200.0)
            sp_f.setFont(mono)
            sp_f.setMinimumWidth(110)
            sp_f.setKeyboardTracking(False)            # 치는 도중이 아니라 Enter·칸 나가기·화살표 때만 계산
            g.addWidget(chk, k + 1, 0)
            g.addWidget(sp_f, k + 1, 1)
            chk.toggled.connect(self._update)
            sp_f.valueChanged.connect(self._update)
            self.rows.append((chk, sp_f))
        g.setRowStretch(4, 1)
        body.addLayout(g)
        self.plot = ImpulsePlot()
        self.plot.setMinimumWidth(220)
        self.plot.setToolTip('성형기 임펄스 (세기 / 시각 ms, 1 ms 칸). 마지막 임펄스 시각 = 지연 시간')
        body.addWidget(self.plot, 1)
        v.addLayout(body)
        self.lb_sum = QtWidgets.QLabel(wordWrap=True)
        self.lb_sum.setTextFormat(QtCore.Qt.RichText)
        self.lb_sum.setToolTip(GRID_TIP)
        v.addWidget(self.lb_sum)
        row = QtWidgets.QHBoxLayout()
        self.btn_apply = QtWidgets.QPushButton('적용', clicked=self.on_apply)
        self.btn_apply.setToolTip('바꾼 성형기를 보냄 (로봇이 멈춰 있을 때만).')
        self.btn_apply.setMinimumWidth(70)
        self.btn_apply.setStyleSheet('QPushButton{font-weight:bold;}')
        self.lb_state = QtWidgets.QLabel()
        self.lb_state.setTextFormat(QtCore.Qt.RichText)
        row.addWidget(self.btn_apply)
        row.addWidget(self.lb_state, 1)
        v.addLayout(row)
        self.grp.idToggled.connect(lambda *_: self._update())

    # ---------- 값 ----------
    def set_config(self, method, rows, apply=True):
        """rows = [(켬, Hz)] 3 개."""
        self.grp.button(METHODS.index(method)).setChecked(True)
        for (chk, sp_f), (use, f) in zip(self.rows, rows):
            chk.setChecked(use)
            sp_f.setValue(f)
        if apply:
            self.applied = self.config()
        self._update()

    def config(self):
        return METHODS[self.grp.checkedId()], tuple((c.isChecked(), round(f.value(), 3)) for c, f in self.rows)

    def label(self, cfg=None):
        return label(*(cfg or self.config()))

    def on_apply(self):
        self.pending = self.config()
        self.apply_requested.emit(self.pending)
        self._update()

    def mark_applied(self, cfg):
        """서버가 받아들임 (또는 시스템 꺼짐: 바로)."""
        self.applied = (cfg[0], tuple((bool(u), round(float(f), 3)) for u, f in cfg[1]))
        self.pending = None
        self._update()

    def reject_pending(self):
        self.pending = None
        self._update()

    def _update(self):
        if self.grp.checkedId() < 0:
            return
        method, rows = self.config()
        shaped = method != 'Unshaped'
        for chk, sp_f in self.rows:
            chk.setEnabled(shaped)
            sp_f.setEnabled(shaped and chk.isChecked())
        res = build(method, rows)
        self.plot.set_result(res)
        n = len(res['t'])
        if n == 1:
            self.lb_sum.setText('임펄스 1 개 · 지연 시간 0 ms')
        else:
            text = f'임펄스 <b>{n}</b> 개 · 지연 시간 <b>{ms(res["t"][-1])} ms</b>'
            bad = [(f, r) for f, r in res['resid'].items() if r > WARN_RESIDUAL]
            if bad:
                text += (f'<br><span style="color:{WARN}">⚠ 1 ms 칸에 맞추느라 ' +
                         ', '.join(f'{f:g} Hz 에서 진동 {r * 100:.0f} %' for f, r in bad) + ' 남음</span>')
            self.lb_sum.setText(text)
        if self.pending is not None and self.pending == self.config():
            self.lb_state.setText(f'<span style="color:{GREY}">● 보내는 중… (서버 답을 기다림)</span>')
            self.btn_apply.setEnabled(False)
        elif self.applied == self.config():
            self.lb_state.setText(f'<span style="color:{OK}">● 적용 중: <b>{self.label()}</b></span>')
            self.btn_apply.setEnabled(False)
        else:
            self.lb_state.setText(f'<span style="color:{WARN}">● 바꾼 값 있음 — [적용] 을 눌러야 반영 (멈춰 있을 때만)</span>'
                                  f'<br><span style="color:{GREY}">지금: {self.label(self.applied) if self.applied else "없음"}</span>')
            self.btn_apply.setEnabled(True)
