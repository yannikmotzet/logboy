from nicegui import ui
import os
import yaml
from logboy.logboy_controller import LogboyController


class LogboyGUI:
    def __init__(self, controller):
        self.controller = controller
        self.is_recording = False
        self.is_paused = False
        self.config = None

        # Resolve assets directory
        ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
        paths = ament_prefix_path.split(os.pathsep)
        if not paths or not paths[0]:
            raise ValueError("AMENT_PREFIX_PATH is not set or invalid.")
        self.assets_dir = os.path.join(paths[0], 'share', 'logboy', 'assets')

        self._build_ui()

    def _build_ui(self):
        with ui.row().classes('items-center justify-center w-full mt-4'):
            self.record_btn = ui.image(self._asset('rec-button.png')).classes('w-24 h-24 cursor-pointer')
            self.record_btn.on('click', self.start_recording)

            self.pause_btn = ui.image(self._asset('pause.png')).classes('w-24 h-24 cursor-pointer opacity-30')
            self.pause_btn.on('click', self.pause_recording)

            self.stop_btn = ui.image(self._asset('stop-button.png')).classes('w-24 h-24 cursor-pointer opacity-30')
            self.stop_btn.on('click', self.stop_recording)

        self.status_label = ui.label('Status: Ready').classes('text-lg text-center w-full mt-2')

        with ui.row().classes('items-center justify-center w-full mt-4 px-4 gap-4'):
            self.yaml_label = ui.label('No config loaded').classes('text-sm text-gray-500 italic')
            ui.upload(
                label='Load YAML config',
                auto_upload=True,
                on_upload=self._on_yaml_upload,
            ).classes('max-w-xs').props('flat bordered accept=".yaml,.yml"')

        # Timer for blinking effect
        self._blink_state = False
        self.blink_timer = ui.timer(1.0, self._blink_record, active=False)
        self.blink_pause_timer = ui.timer(0.25, self._blink_pause, active=False)

    # --- Asset helper ---

    def _asset(self, filename):
        path = os.path.join(self.assets_dir, filename)
        if os.path.exists(path):
            return path
        return 'https://placehold.co/100x100/gray/gray'  # fallback placeholder

    # --- Config ---

    async def _on_yaml_upload(self, e):
        try:
            raw = await e.file.read()
            self.config = yaml.safe_load(raw)
            self.yaml_label.set_text(f'Loaded: {e.file.name}')
            self.yaml_label.classes(remove='text-gray-500 text-red-500', add='text-green-600')
        except Exception as ex:
            self.config = None
            self.yaml_label.set_text(f'Error: {ex}')
            self.yaml_label.classes(remove='text-gray-500 text-green-600', add='text-red-500')

    # --- Blink helpers ---

    def _blink_record(self):
        self._blink_state = not self._blink_state
        src = self._asset('rec-button_inactive_2.png') if self._blink_state else self._asset('rec-button.png')
        self.record_btn.set_source(src)

    def _blink_pause(self):
        self._blink_state = not self._blink_state
        src = self._asset('circular.png') if self._blink_state else self._asset('pause.png')
        self.pause_btn.set_source(src)

    # --- Button state helpers ---

    def _set_enabled(self, element, enabled: bool):
        if enabled:
            element.classes(remove='opacity-30 pointer-events-none')
        else:
            element.classes(add='opacity-30 pointer-events-none')

    # --- Actions ---

    def start_recording(self):
        if self.config is None:
            ui.notify('Please load a YAML config file first.', type='warning')
            return

        self.controller.configure_recorder(self.config)
        self.controller.start_recording()

        self.is_recording = True
        self.is_paused = False
        self.status_label.set_text('Status: Recording')

        self._set_enabled(self.record_btn, False)
        self._set_enabled(self.stop_btn, True)
        self._set_enabled(self.pause_btn, True)

        self.blink_timer.activate()

    def stop_recording(self):
        self.controller.stop_recording()

        self.is_recording = False
        self.is_paused = False
        self.status_label.set_text('Status: Stopped')

        self.blink_timer.deactivate()
        self.blink_pause_timer.deactivate()

        self.record_btn.set_source(self._asset('rec-button.png'))
        self.pause_btn.set_source(self._asset('pause.png'))

        self._set_enabled(self.record_btn, True)
        self._set_enabled(self.stop_btn, False)
        self._set_enabled(self.pause_btn, False)

    def pause_recording(self):
        if not self.is_paused:
            self.controller.pause_recording()

            self.is_recording = False
            self.is_paused = True
            self.status_label.set_text('Status: Paused')

            self.blink_timer.deactivate()
            self.record_btn.set_source(self._asset('rec-button.png'))

            self.blink_pause_timer.activate()
        else:
            self.controller.resume_recording()

            self.is_recording = True
            self.is_paused = False
            self.status_label.set_text('Status: Recording Resumed')

            self.blink_pause_timer.deactivate()
            self.pause_btn.set_source(self._asset('pause.png'))

            self.blink_timer.activate()


def main():
    controller = LogboyController()

    @ui.page('/')
    def index():
        LogboyGUI(controller)

    ui.run(title='logboy', reload=False)