#!/usr/bin/env python3

import json

from cv_bridge import CvBridge
from PyQt5 import QtCore, QtGui, QtWidgets
from rqt_gui_py.plugin import Plugin
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String


class HandeyeRqtPlugin(Plugin):
    """Responsive rqt panel for hand-eye sample collection."""

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName('HandeyeRqtPlugin')
        self.bridge = CvBridge()
        self.latest_qimage = None
        self.status = {}
        self._publishers = {}

        self.widget = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(self.widget)
        self.image_label = QtWidgets.QLabel('Waiting for /charuco/result')
        self.image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.image_label.setMinimumSize(640, 360)
        self.image_label.setStyleSheet('background: #171a1f; color: #b8c0cc;')
        layout.addWidget(self.image_label, 1)

        panel = QtWidgets.QVBoxLayout()
        self.mode_label = QtWidgets.QLabel('Mode: waiting')
        self.ready_label = QtWidgets.QLabel('Camera -   Board -   Robot -')
        self.pair_label = QtWidgets.QLabel('Pair delta: -')
        self.samples_label = QtWidgets.QLabel('Samples: 0 / 15')
        self.unique_label = QtWidgets.QLabel('Nearest saved pose: -')
        self.quality_label = QtWidgets.QLabel('Quality: - / 100')
        self.quality_label.setStyleSheet('font-size: 22px; font-weight: 600;')
        self.quality_parts_label = QtWidgets.QLabel(
            'Board -/40   Novelty -/40   Coverage -/20')
        self.quality_tip_label = QtWidgets.QLabel('')
        self.quality_tip_label.setWordWrap(True)
        self.message_label = QtWidgets.QLabel('Waiting for calibration nodes')
        self.message_label.setWordWrap(True)
        for label in (self.mode_label, self.ready_label, self.pair_label,
                      self.samples_label, self.unique_label, self.quality_label,
                      self.quality_parts_label, self.quality_tip_label,
                      self.message_label):
            panel.addWidget(label)

        self.capture_button = QtWidgets.QPushButton('Capture sample')
        self.remove_button = QtWidgets.QPushButton('Remove last')
        self.calculate_button = QtWidgets.QPushButton('Validate and save')
        self.exit_button = QtWidgets.QPushButton('Exit calibration')
        self.capture_button.clicked.connect(lambda: self.publish_command('/handeye/capture'))
        self.remove_button.clicked.connect(lambda: self.publish_command('/handeye/remove'))
        self.calculate_button.clicked.connect(lambda: self.publish_command('/handeye/calculate'))
        self.exit_button.clicked.connect(lambda: self.publish_command('/handeye/exit'))
        for button in (self.capture_button, self.remove_button,
                       self.calculate_button, self.exit_button):
            button.setMinimumHeight(42)
            panel.addWidget(button)
        panel.addStretch(1)
        layout.addLayout(panel)

        context.add_widget(self.widget)
        self.node = context.node if hasattr(context, 'node') else None
        self.image_sub = self._create_subscription(Image, '/charuco/result', self.image_callback)
        self.status_sub = self._create_subscription(String, '/handeye/status', self.status_callback)
        self.refresh_timer = QtCore.QTimer(self.widget)
        self.refresh_timer.timeout.connect(self.refresh_image)
        self.refresh_timer.start(100)

    def _create_subscription(self, message_type, topic, callback):
        if self.node is not None:
            return self.node.create_subscription(message_type, topic, callback, 10)
        return None

    def _publisher(self, topic):
        publisher = self._publishers.get(topic)
        if publisher is None and self.node is not None:
            publisher = self.node.create_publisher(Empty, topic, 10)
            self._publishers[topic] = publisher
        return publisher

    def publish_command(self, topic):
        publisher = self._publisher(topic)
        if publisher is not None:
            publisher.publish(Empty())

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
            height, width, channels = image.shape
            qimage = QtGui.QImage(
                image.data, width, height, channels * width,
                QtGui.QImage.Format_RGB888).copy()
            self.latest_qimage = qimage
        except Exception:
            self.latest_qimage = None

    def status_callback(self, message):
        try:
            self.status = json.loads(message.data)
        except (TypeError, ValueError):
            self.status = {'message': message.data}
    def refresh_status(self):
        self.mode_label.setText(
            f'{self.status.get("camera_name", "-")} | {self.status.get("mode", "-")}')
        self.ready_label.setText(
            'Camera {camera}   Board {board}   Robot {robot}'.format(
                camera='OK' if self.status.get('camera') else 'WAIT',
                board='OK' if self.status.get('board') else 'WAIT',
                robot='OK' if self.status.get('robot') else 'WAIT'))
        delta = self.status.get('pair_delta_sec', float('inf'))
        self.pair_label.setText(
            f'Pair delta: {delta * 1000:.0f} ms' if delta != float('inf') else 'Pair delta: -')
        self.samples_label.setText(
            f'Samples: {self.status.get("samples", 0)} / {self.status.get("min_samples", 15)}')
        translation = self.status.get('nearest_translation_m', float('inf'))
        rotation = self.status.get('nearest_rotation_deg', float('inf'))
        if translation == float('inf'):
            self.unique_label.setText('Nearest saved pose: first sample')
        else:
            self.unique_label.setText(
                f'Nearest saved pose: {translation * 1000:.0f} mm / {rotation:.1f} deg')
        quality = self.status.get('capture_quality', {})
        total = quality.get('total', 0.0)
        verdict = quality.get('verdict', '-')
        quality_colors = {
            'GOOD': '#18864b', 'USABLE': '#a36b00',
            'LOW': '#b64a24', 'NO BOARD': '#b42318',
        }
        self.quality_label.setText(f'Quality: {total:.0f} / 100   {verdict}')
        self.quality_label.setStyleSheet(
            'font-size: 22px; font-weight: 600; color: '
            + quality_colors.get(verdict, '#333333') + ';')
        self.quality_parts_label.setText(
            'Board {board:.0f}/40   Novelty {novelty:.0f}/40   Coverage {coverage:.0f}/20'.format(
                board=quality.get('board', 0.0),
                novelty=quality.get('novelty', 0.0),
                coverage=quality.get('coverage', 0.0)))
        self.quality_tip_label.setText(
            'Tip: ' + ' | '.join(quality.get('suggestions', [])))
        self.message_label.setText(self.status.get('message', ''))
        self.capture_button.setEnabled(self.status.get('can_capture', False))
        self.calculate_button.setEnabled(not self.status.get('finished', False))

    def refresh_image(self):
        self.refresh_status()
        if self.latest_qimage is not None:
            pixmap = QtGui.QPixmap.fromImage(self.latest_qimage)
            self.image_label.setPixmap(pixmap.scaled(
                self.image_label.size(), QtCore.Qt.KeepAspectRatio,
                QtCore.Qt.SmoothTransformation))

    def shutdown_plugin(self):
        self.refresh_timer.stop()
        if self.node is not None:
            for subscription in (self.image_sub, self.status_sub):
                if subscription is not None:
                    self.node.destroy_subscription(subscription)
            for publisher in self._publishers.values():
                self.node.destroy_publisher(publisher)

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass


def main():
    from rqt_gui.main import Main
    Main().main()
