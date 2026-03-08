from nicegui import ui
import os
import yaml
import argparse
from datetime import datetime
from logboy.logboy_controller import LogboyController
from logboy.logboy_stats import TopicSnapshot


class LogboyGUI:
    def __init__(self, controller: LogboyController, config: dict):
        self.controller = controller
        self.config = config
        self.is_paused = False

        ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
        paths = ament_prefix_path.split(os.pathsep)
        if not paths or not paths[0]:
            raise ValueError("AMENT_PREFIX_PATH is not set or invalid.")
        self.assets_dir = os.path.join(paths[0], 'share', 'logboy', 'assets')

        self._blink_state = False
        self._build_ui()

    # ── Build ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Transport controls
        with ui.row().classes('items-center justify-center w-full mt-4 gap-4'):
            self.record_btn = ui.image(self._asset('rec-button.png')).classes('w-24 h-24 cursor-pointer')
            self.record_btn.on('click', self.start_recording)

            self.pause_btn = ui.image(self._asset('pause.png')).classes('w-24 h-24 cursor-pointer opacity-30 pointer-events-none')
            self.pause_btn.on('click', self.toggle_pause)

            self.stop_btn = ui.image(self._asset('stop-button.png')).classes('w-24 h-24 cursor-pointer opacity-30 pointer-events-none')
            self.stop_btn.on('click', self.stop_recording)

        self.status_label = ui.label('Status: Ready').classes('text-lg text-center w-full mt-2')

        # Topic monitor table
        ui.separator().classes('my-4')
        columns = [
            {'name': 'name',         'label': 'Topic',    'field': 'name',         'align': 'left',  'sortable': True},
            {'name': 'expected_fps', 'label': 'Exp. FPS', 'field': 'expected_fps', 'align': 'right', 'sortable': True},
            {'name': 'fps',          'label': 'FPS',      'field': 'fps',          'align': 'right', 'sortable': True},
            {'name': 'drops',        'label': 'Drops',    'field': 'drops',        'align': 'right', 'sortable': True},
            {'name': 'total_msgs',   'label': 'Msgs',     'field': 'total_msgs',   'align': 'right', 'sortable': True},
            {'name': 'age',          'label': 'Age (s)',  'field': 'age',          'align': 'right', 'sortable': True},
        ]
        self.table = ui.table(columns=columns, rows=[], row_key='name').classes('w-full')

        self.table.add_slot('body-cell-expected_fps', '''
            <q-td :props="props">
                {{ props.value > 0 ? props.value.toFixed(1) : '—' }}
            </q-td>
        ''')
        self.table.add_slot('body-cell-fps', '''
            <q-td :props="props">
                <span :style="{ color:
                    props.row.active === false ? '#6b7280' :
                    props.row.age > 5          ? '#6b7280' :
                    props.value < 1            ? '#ef4444' : '#22c55e' }">
                    {{ props.row.active ? props.value.toFixed(1) : '—' }}
                </span>
            </q-td>
        ''')
        self.table.add_slot('body-cell-drops', '''
            <q-td :props="props">
                <span :style="{ color: props.value > 0 ? '#ef4444' : '#22c55e' }">
                    {{ props.value }}
                </span>
            </q-td>
        ''')
        self.table.add_slot('body-cell-age', '''
            <q-td :props="props">
                <span :style="{ color:
                    props.row.active === false                                          ? '#6b7280' :
                    props.row.age > 5                                                  ? '#ef4444' :
                    props.row.expected_fps > 0 && props.row.age > 1/props.row.expected_fps*3 ? '#eab308' :
                                                                                         '#22c55e' }">
                    {{ props.row.active ? props.value.toFixed(1) : '—' }}
                </span>
            </q-td>
        ''')

        # Timers
        self.blink_timer      = ui.timer(1.0,    self._blink_record, active=False)
        self.blink_pause_timer = ui.timer(0.25,  self._blink_pause,  active=False)
        self.monitor_timer    = ui.timer(1.0,    self._refresh_table, active=True)

    # ── Transport controls ───────────────────────────────────────────────────

    def start_recording(self):
        self.controller.start_recording()
        self.is_paused = False
        self.status_label.set_text('Status: Recording')
        self._set_enabled(self.record_btn, False)
        self._set_enabled(self.stop_btn,   True)
        self._set_enabled(self.pause_btn,  True)
        self.blink_timer.activate()

    def stop_recording(self):
        self.controller.stop_recording()
        self.is_paused = False
        self.status_label.set_text('Status: Stopped')
        self.blink_timer.deactivate()
        self.blink_pause_timer.deactivate()
        self.record_btn.set_source(self._asset('rec-button.png'))
        self.pause_btn.set_source(self._asset('pause.png'))
        self._set_enabled(self.record_btn, True)
        self._set_enabled(self.stop_btn,   False)
        self._set_enabled(self.pause_btn,  False)

    def toggle_pause(self):
        if not self.is_paused:
            self.controller.pause_recording()
            self.is_paused = True
            self.status_label.set_text('Status: Paused')
            self.blink_timer.deactivate()
            self.record_btn.set_source(self._asset('rec-button.png'))
            self.blink_pause_timer.activate()
        else:
            self.controller.resume_recording()
            self.is_paused = False
            self.status_label.set_text('Status: Recording')
            self.blink_pause_timer.deactivate()
            self.pause_btn.set_source(self._asset('pause.png'))
            self.blink_timer.activate()

    # ── Monitor ──────────────────────────────────────────────────────────────

    def _refresh_table(self):
        snapshots: list[TopicSnapshot] = self.controller.get_stats()
        self.table.rows = [self._snapshot_to_row(s) for s in sorted(snapshots, key=lambda s: s.name)]
        self.table.update()

    @staticmethod
    def _snapshot_to_row(s: TopicSnapshot) -> dict:
        return {
            'name':         s.name,
            'expected_fps': s.expected_fps,
            'fps':          s.fps,
            'drops':        s.drops,
            'total_msgs':   s.total_msgs,
            'age':          s.age,
            'active':       s.first_seen is not None,
        }

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _asset(self, filename: str) -> str:
        path = os.path.join(self.assets_dir, filename)
        return path if os.path.exists(path) else 'https://placehold.co/100x100/gray/gray'

    def _set_enabled(self, element, enabled: bool):
        if enabled:
            element.classes(remove='opacity-30 pointer-events-none')
        else:
            element.classes(add='opacity-30 pointer-events-none')

    def _blink_record(self):
        self._blink_state = not self._blink_state
        src = self._asset('rec-button_inactive_2.png') if self._blink_state else self._asset('rec-button.png')
        self.record_btn.set_source(src)

    def _blink_pause(self):
        self._blink_state = not self._blink_state
        src = self._asset('circular.png') if self._blink_state else self._asset('pause.png')
        self.pause_btn.set_source(src)


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Logboy GUI')
    parser.add_argument('--config', required=True, help='Path to YAML config file')
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    controller = LogboyController()
    controller.configure_recorder(config)   # ← einmalig hier, nicht in start_recording

    @ui.page('/')
    def index():
        LogboyGUI(controller, config)

    ui.run(title='logboy', reload=False)


if __name__ == '__main__':
    main()