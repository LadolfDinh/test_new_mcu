# -*- coding: utf-8 -*-
"""Mau va kieu ve dung chung (bang mau da kiem tra mu mau)."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BLUE, ORANGE, AQUA = '#257de0', '#f47230', '#00b894'
TEXT, TEXT2, GRID = '#191919', '#666666', '#e7e7e7'
SEQ_BLUE = ['#cce3fd', '#9dc8f7', '#6aabf1', '#338ced', '#2370cd', '#1757ac', '#0e3d7e']


def setup():
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 9,
        'axes.edgecolor': GRID, 'axes.labelcolor': TEXT2, 'axes.titlecolor': TEXT,
        'axes.titlesize': 10, 'axes.titleweight': 'bold',
        'xtick.color': TEXT2, 'ytick.color': TEXT2,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
        'axes.spines.top': False, 'axes.spines.right': False,
        'legend.frameon': False, 'legend.fontsize': 8, 'lines.linewidth': 1.6,
        'figure.facecolor': 'white', 'axes.facecolor': 'white',
    })
