import sys
from .trainer import *
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal

class TrainingWindow(QtWidgets.QMainWindow):
    start_training_signal = pyqtSignal()
    stop_training_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.trainer = None
        self.init_ui()

    def init_ui(self):
        self.init_window()
        self.init_layouts()
        self.init_components()

    def init_trainer(self):
        self.trainer = TrainingThread()
        self.start_training_signal.connect(self.trainer.start_training)
        self.stop_training_signal.connect(self.trainer.stop_training)
        self.trainer.update_scores.connect(self.update_scores)

    def init_window(self):
        self.setWindowTitle("Linkage Trainer")
        self.setGeometry(100, 100, 400, 400)
        self.show()

    def init_layouts(self):
        self.central_widget = QtWidgets.QWidget()
        self.setCentralWidget(self.central_widget)
        self.main_layout = QtWidgets.QVBoxLayout()
        self.central_widget.setLayout(self.main_layout)

        self.algo_layout = QtWidgets.QHBoxLayout()
        self.main_layout.addLayout(self.algo_layout)
        self.btn_layout = QtWidgets.QHBoxLayout()
        self.main_layout.addLayout(self.btn_layout)
        self.data_layout = QtWidgets.QVBoxLayout()
        self.main_layout.addLayout(self.data_layout)

    def init_components(self):
        self.init_algo_radio_btns()
        self.init_training_btns()
        self.init_data_labels()

    def init_algo_radio_btns(self):
        self.algo_grp = QtWidgets.QButtonGroup()

        self.anneal_radio = QtWidgets.QRadioButton("Simulated Annealing")
        self.anneal_radio.setChecked(True)
        self.algo_grp.addButton(self.anneal_radio)
        self.algo_layout.addWidget(self.anneal_radio)

        self.ga_radio = QtWidgets.QRadioButton("Genetic Algorithm")
        self.algo_grp.addButton(self.ga_radio)
        self.algo_layout.addWidget(self.ga_radio)

    def init_training_btns(self):
        self.start_training_btn = QtWidgets.QPushButton("Start Training")
        self.btn_layout.addWidget(self.start_training_btn)
        self.start_training_btn.clicked.connect(self.on_start_training_btn_clicked)

        self.stop_training_btn = QtWidgets.QPushButton("Stop Training")
        self.btn_layout.addWidget(self.stop_training_btn)
        self.stop_training_btn.clicked.connect(self.on_stop_training_btn_clicked)
        self.stop_training_btn.setEnabled(False)

    def init_data_labels(self):
        scores = load_scores_json()
        rounded_scores = {k: round(v, 2) for k, v in scores.items()}
        self.high_score_label = QtWidgets.QLabel(f"Highest Score: {round(max(rounded_scores.values(), default=0), 2)}")
        self.current_score_label = QtWidgets.QLabel("Current Score: 0")
        self.all_scores_label = QtWidgets.QLabel(f"All Scores: {list(rounded_scores.values())}")
        self.data_layout.addWidget(self.high_score_label)
        self.data_layout.addWidget(self.current_score_label)
        self.data_layout.addWidget(self.all_scores_label)

    def update_scores(self, high_score, scores):
        rounded_scores = [round(score, 2) for score in scores]
        self.high_score_label.setText(f"Highest Score: {round(high_score, 2)}")
        self.current_score_label.setText(f"Current Score: {rounded_scores[-1]}")
        self.all_scores_label.setText(f"All Scores: {rounded_scores}")

    def on_start_training_btn_clicked(self):
        algorithm = 'SA' if self.sa_radio.isChecked() else 'GA'
        self.trainer = TrainingThread(algorithm) 
        self.start_training_signal.connect(self.trainer.start_training)
        self.stop_training_signal.connect(self.trainer.stop_training)
        self.trainer.update_scores.connect(self.update_scores)

        self.start_training_signal.emit(algorithm) 
        self.enable_disable_training_btns(True)

    def on_stop_training_btn_clicked(self):
        self.stop_training_signal.emit()
        self.enable_disable_training_btns(False)

    def enable_disable_training_btns(self, is_training):
        self.start_training_btn.setEnabled(not is_training)
        self.stop_training_btn.setEnabled(is_training)

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = TrainingWindow()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()