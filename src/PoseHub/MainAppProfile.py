import sys
import cProfile, pstats
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
    tool_names = ["object_1", "object_2", "ref_1"]
    config = NetworkConfig("127.0.0.1", "5555", "5556", "Camera")

    main_window = PoseGraphGUI(tool_names, config, ref_frame="Camera")
    main_window.show()

    # Start the profiler here.
    profiler = cProfile.Profile()
    profiler.enable()

    exit_code = app.exec_()

    # Disable profiling once the GUI is closed
    profiler.disable()

    # Print the top 20 functions by cumulative time to stdout.
    stats = pstats.Stats(profiler).sort_stats("cumulative")
    stats.print_stats(20)

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
