#!/usr/bin/env python3

import json
import sys
import platform
import ctypes

import matplotlib.pyplot as plt
import numpy as np
import qtmodern.styles
import qtmodern.windows
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5 import QtCore
from PyQt5.QtGui import QPixmap
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import (
    QApplication,
    QDialog,
    QHBoxLayout,
    QInputDialog,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QSplashScreen,
    QVBoxLayout,
)

from helpers import PATH_OBSTACLES, LineBuilder, create_obstacles, in2m


class Window(QDialog):
    def __init__(self, parent=None):
        super(Window, self).__init__(parent)

        plt.style.use("ggplot")

        # Field Canvas
        self.canvas = FigureCanvas(plt.figure())
        self.canvas.axes = self.canvas.figure.add_subplot(111)

        plt.axis("equal")
        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.xlim(-1, in2m(360) + 1)
        plt.ylim(-1, in2m(180) + 1)

        (self.line,) = self.canvas.axes.plot(
            [in2m(30)], [in2m(30)], marker="o", color="cornflowerblue"
        )  # Start pos
        self.lb = LineBuilder(self.line)
        self.toolbar = NavigationToolbar(self.canvas, self)

        self.canvas.setFocusPolicy(QtCore.Qt.ClickFocus)
        self.canvas.setFocus()

        # Obstacles List
        obstaclesList = QListWidget()
        for obstacle in PATH_OBSTACLES:
            obstaclesList.addItem(QListWidgetItem(obstacle))
        obstaclesList.itemClicked.connect(self.add_obstacles)

        # Save Button
        saveButton = QPushButton()
        saveButton.setText("Save to JSON")
        saveButton.clicked.connect(self.write_json)

        # Layout
        main_layout = QHBoxLayout()

        plot_layout = QVBoxLayout()
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)

        settings_layout = QVBoxLayout()
        settings_layout.addWidget(obstaclesList)
        settings_layout.addWidget(saveButton)

        main_layout.addLayout(settings_layout)
        main_layout.addLayout(plot_layout)

        self.setLayout(main_layout)

        self.canvas.draw()

    def add_obstacles(self, item):
        obstacles = create_obstacles(item.text())
        for obj in self.canvas.axes.findobj(match=plt.Circle):
            obj.remove()

        for obx, oby, obr in obstacles:
            self.canvas.axes.add_artist(plt.Circle((obx, oby), obr, color="r"))
        self.canvas.draw()

    def write_json(self):
        name, ok = QInputDialog.getText(self, "Name Input", "Enter a JSON name")

        if ok:
            output = []
            for (x, y) in zip(self.lb.xs, self.lb.ys):
                output.append((x, y))

            with open("./init_traj/{}.json".format(str(name)), "w") as outfile:
                json.dump(output, outfile, indent=4)

            msg = QMessageBox()
            msg.setIcon(QMessageBox.Information)
            msg.setText("The initialization was saved as {}.json".format(name))
            msg.setWindowTitle("Saved")
            msg.setStandardButtons(QMessageBox.Ok)
            msg.exec_()


def run():
    app = QApplication(sys.argv)
    qtmodern.styles.dark(app)

    app.setWindowIcon(QIcon("resources/icon.ico"))
    if platform.system() == "Windows":
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(
            "quikplan"
        )  # App ID for taskbar icon

    splash = QSplashScreen(
        QPixmap("resources/quikplan.png").scaledToHeight(
            400, QtCore.Qt.TransformationMode.SmoothTransformation
        )
    )
    splash.setWindowFlags(
        QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.FramelessWindowHint
    )
    splash.show()

    main = Window()

    def showWindow():
        splash.close()
        main.show()

    QtCore.QTimer.singleShot(1000, showWindow)

    sys.exit(app.exec_())


if __name__ == "__main__":
    run()
