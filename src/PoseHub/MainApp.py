import sys
from PyQt5 import QtCore, QtWidgets
from posehub_gui import PoseGraphGUI
from utils import NetworkConfig


def main():
    app = QtWidgets.QApplication(sys.argv)

    # Set a dark theme stylesheet
    app.setStyleSheet(
        """
        QWidget {
            background-color: #2b2b2b;
            color: white;
        }
        QLineEdit, QTextEdit, QPlainTextEdit {
            background-color: #3c3c3c;
            color: white;
        }
        QPushButton {
            background-color: #3c3c3c;
            color: white;
            border: 1px solid #555;
            padding: 5px;
        }
        QToolBar {
            background-color: #2b2b2b;
            spacing: 6px;
        }
        QMenuBar, QMenu {
            background-color: #2b2b2b;
            color: white;
        }
        QHeaderView::section {
            background-color: #3c3c3c;
            color: white;
        }
    """
    )
    # tool_names = ["object_1", "object_2", "ref_1"]
    # config = NetworkConfig("127.0.0.1", "5555", "5556", "Camera")

    tool_names = ["Probe", "StaticRef", "Anatomy"]
    config = NetworkConfig("192.168.0.105", "5586", "5587", "Holo_1")
    main_window = PoseGraphGUI(
        tool_names,
        config,
        ref_frame="Camera",
        filename="scenegraph_0411_quant_square_1",
    )
    main_window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
