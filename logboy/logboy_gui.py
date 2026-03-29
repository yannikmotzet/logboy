from nicegui import ui, app
import os
import shutil
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

        self._blink_state = False
        self._ui_recording = False   # last known UI state
        self._ui_paused    = False
        self._build_ui()

    # ── Build ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Header with logo and dark mode toggle
        with ui.row().classes('items-center w-full mt-4 px-4'):
            ui.element('div').classes('flex-1')
            with ui.row().classes('items-center gap-3'):
                ui.image('/assets/logboy_logo.png').classes('w-12 h-12')
                ui.label('logboy').classes('text-3xl font-bold')
            with ui.row().classes('flex-1 justify-end items-center gap-3'):
                self.clock_label = ui.label().classes('text-sm font-mono text-gray-400')
                self.dark = ui.dark_mode(value=True)
                self.dark_btn = ui.button(icon='dark_mode', on_click=self._toggle_dark).props('flat round')

        # Transport controls
        with ui.row().classes('items-center justify-center w-full mt-4 gap-2'):
            self.record_btn = ui.button(icon='fiber_manual_record', on_click=self.start_recording) \
                .props('round flat size="xl" color="red"')
            self.stop_btn = ui.button(icon='stop', on_click=self.stop_recording) \
                .props('round flat size="xl" color="grey-6"')
            self.stop_btn.set_visibility(False)
            self.pause_btn = ui.button(icon='pause', on_click=self.toggle_pause) \
                .props('round flat size="xl" color="grey" disable')

        with ui.row().classes('items-center justify-center w-full mt-2 gap-1'):
            self.rec_indicator = ui.icon('fiber_manual_record', size='sm').classes('text-transparent')
            self.status_label = ui.label('Ready').classes('text-lg')

        # Recording info panel
        with ui.row().classes('items-center justify-center w-full gap-8 mt-1') as self.rec_info_row:
            with ui.column().classes('items-center gap-0 w-32'):
                ui.label('Robot').classes('text-xs text-gray-400')
                ui.label(self.config.get('robot_name', '—')).classes('text-sm font-mono text-center')
            with ui.column().classes('items-center gap-0 w-48'):
                ui.label('Storage').classes('text-xs text-gray-400')
                self.rec_path_label = ui.label(os.path.expanduser(self.config.get('storage_path', '—'))).classes('text-sm font-mono text-center')
            with ui.column().classes('items-center gap-0 w-64'):
                ui.label('Recording').classes('text-xs text-gray-400')
                self.rec_name_label = ui.label('—').classes('text-sm font-mono text-center')
            with ui.column().classes('items-center gap-0 w-24'):
                ui.label('Elapsed').classes('text-xs text-gray-400')
                self.elapsed_label = ui.label('—').classes('text-sm font-mono text-center')
            with ui.column().classes('items-center gap-0 w-24'):
                ui.label('Size').classes('text-xs text-gray-400')
                self.size_label = ui.label('—').classes('text-sm font-mono text-center')
            with ui.column().classes('items-center gap-0 w-36'):
                ui.label('Free').classes('text-xs text-gray-400')
                self.free_label = ui.label('—').classes('text-sm font-mono text-center')
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
        self.status_label.set_text('Recording')
        self.record_btn.set_visibility(False)
        self.stop_btn.set_visibility(True)
        self._set_enabled(self.pause_btn, True)
        self.rec_indicator.classes('text-red-500', remove='text-transparent')
        self.blink_timer.activate()

    def stop_recording(self):
        self.controller.stop_recording()
        self.is_paused = False
        self.status_label.set_text('Stopped')
        self.blink_timer.deactivate()
        self.blink_pause_timer.deactivate()
        self.pause_btn.props('icon=pause color=grey')
        self.stop_btn.set_visibility(False)
        self.record_btn.set_visibility(True)
        self._set_enabled(self.pause_btn, False)
        self.rec_indicator.props('name=fiber_manual_record')
        self.rec_indicator.classes('text-transparent', remove='text-red-500 text-orange-500')

    def toggle_pause(self):
        if not self.is_paused:
            self.controller.pause_recording()
            self.is_paused = True
            self.status_label.set_text('Paused')
            self.blink_timer.deactivate()
            self.rec_indicator.classes('text-transparent', remove='text-red-500 text-orange-500')
            self.rec_indicator.props('name=pause')
            self.pause_btn.props('icon=play_arrow color=grey-6')
            self.blink_pause_timer.activate()
        else:
            self.controller.resume_recording()
            self.is_paused = False
            self.status_label.set_text('Recording')
            self.blink_pause_timer.deactivate()
            self.rec_indicator.classes('text-transparent', remove='text-orange-500')
            self.rec_indicator.props('name=fiber_manual_record')
            self.pause_btn.props('icon=pause color=grey')
            self.rec_indicator.classes('text-red-500', remove='text-transparent')
            self.blink_timer.activate()

    # ── Monitor ──────────────────────────────────────────────────────────────

    def _refresh_table(self):
        snapshots: list[TopicSnapshot] = self.controller.get_stats()
        self.table.rows = [self._snapshot_to_row(s) for s in sorted(snapshots, key=lambda s: s.name)]
        self.table.update()
        self._sync_state()
        self._refresh_rec_info()
        self._refresh_free_space()
        self.clock_label.set_text(datetime.now().strftime('%H:%M:%S'))

    def _sync_state(self):
        """Sync button/status state from the controller (handles multi-tab and page reload)."""
        is_recording = self.controller.get_bag_path() is not None
        is_paused    = self.controller.is_paused()

        if is_recording == self._ui_recording and is_paused == self._ui_paused:
            return  # nothing changed

        if not is_recording:
            self.status_label.set_text('Stopped' if self._ui_recording else 'Ready')
            self.record_btn.set_visibility(True)
            self.stop_btn.set_visibility(False)
            self._set_enabled(self.pause_btn, False)
            self.blink_timer.deactivate()
            self.blink_pause_timer.deactivate()
            self.pause_btn.props('icon=pause color=grey')
            self.rec_indicator.props('name=fiber_manual_record')
            self.rec_indicator.classes('text-transparent', remove='text-red-500 text-orange-500')
        elif is_paused:
            self.status_label.set_text('Paused')
            self.record_btn.set_visibility(False)
            self.stop_btn.set_visibility(True)
            self._set_enabled(self.pause_btn, True)
            self.blink_timer.deactivate()
            self.rec_indicator.props('name=pause')
            self.pause_btn.props('icon=play_arrow color=grey-6')
            self.rec_indicator.classes('text-transparent', remove='text-red-500')
            self.blink_pause_timer.activate()
        else:
            self.status_label.set_text('Recording')
            self.record_btn.set_visibility(False)
            self.stop_btn.set_visibility(True)
            self._set_enabled(self.pause_btn, True)
            self.blink_pause_timer.deactivate()
            self.rec_indicator.props('name=fiber_manual_record')
            self.pause_btn.props('icon=pause color=grey')
            self.rec_indicator.classes('text-red-500', remove='text-transparent text-orange-500')
            self.blink_timer.activate()

        self._ui_recording = is_recording
        self._ui_paused    = is_paused
        self.is_paused     = is_paused

    def _refresh_rec_info(self):
        elapsed = self.controller.get_elapsed()
        if elapsed is None:
            return
        bag_path = self.controller.get_bag_path()
        h, rem = divmod(int(elapsed), 3600)
        m, s = divmod(rem, 60)
        self.elapsed_label.set_text(f'{h:02d}:{m:02d}:{s:02d}')
        if bag_path:
            self.rec_name_label.set_text(os.path.basename(bag_path))
            self.rec_path_label.set_text(os.path.dirname(bag_path))
            size_bytes = self._bag_size_bytes(bag_path)
            self.size_label.set_text(self._fmt_bytes(size_bytes))
            # update time-left estimate in free label
            if elapsed > 5 and size_bytes > 0:
                storage_path = self.config.get('storage_path', '')
                try:
                    free_bytes = shutil.disk_usage(os.path.expanduser(storage_path)).free
                    rate = size_bytes / elapsed  # bytes/s
                    secs_left = int(free_bytes / rate)
                    h2, r2 = divmod(secs_left, 3600)
                    m2 = r2 // 60
                    self.free_label.set_text(f'{self._fmt_bytes(free_bytes)} (~{h2}h {m2:02d}m)')
                except OSError:
                    pass

    def _refresh_free_space(self):
        """Update free space label when not recording (no rate estimate available)."""
        if self.controller.get_elapsed() is not None:
            return  # handled by _refresh_rec_info
        storage_path = self.config.get('storage_path', '')
        try:
            free_bytes = shutil.disk_usage(os.path.expanduser(storage_path)).free
            self.free_label.set_text(self._fmt_bytes(free_bytes))
        except OSError:
            self.free_label.set_text('—')

    @staticmethod
    def _bag_size_bytes(bag_path: str) -> int:
        if not os.path.isdir(bag_path):
            return 0
        return sum(e.stat().st_size for e in os.scandir(bag_path) if e.is_file())

    @staticmethod
    def _fmt_bytes(total: float) -> str:
        for unit in ('B', 'KB', 'MB', 'GB'):
            if total < 1024:
                return f'{total:.1f} {unit}'
            total /= 1024
        return f'{total:.1f} TB'

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

    def _toggle_dark(self):
        self.dark.toggle()
        self.dark_btn.props('icon=light_mode' if self.dark.value else 'icon=dark_mode')

    def _set_enabled(self, element, enabled: bool):
        if enabled:
            element.props(remove='disable')
        else:
            element.props(add='disable')

    def _blink_record(self):
        self._blink_state = not self._blink_state
        if self._blink_state:
            self.rec_indicator.classes('text-red-500', remove='text-transparent')
        else:
            self.rec_indicator.classes('text-transparent', remove='text-red-500')

    def _blink_pause(self):
        self._blink_state = not self._blink_state
        if self._blink_state:
            self.rec_indicator.classes('text-orange-500', remove='text-transparent')
        else:
            self.rec_indicator.classes('text-transparent', remove='text-orange-500')


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Logboy GUI')
    parser.add_argument('--config', required=True, help='Path to YAML config file')
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    controller = LogboyController()
    controller.configure_recorder(config)   # ← einmalig hier, nicht in start_recording

    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
    assets_dir = os.path.join(ament_prefix_path.split(os.pathsep)[0], 'share', 'logboy', 'assets')
    app.add_static_files('/assets', assets_dir)

    @ui.page('/')
    def index():
        ui.add_head_html('''
        <style>
        #logboy-disconnected {
            display: none;
            position: fixed;
            bottom: 2rem;
            left: 50%;
            transform: translateX(-50%);
            background: #1f2937;
            border: 1px solid #f87171;
            border-radius: 0.75rem;
            padding: 0.75rem 1.5rem;
            z-index: 99999;
            flex-direction: row;
            align-items: center;
            gap: 0.75rem;
            box-shadow: 0 4px 24px rgba(0,0,0,0.6);
        }
        #logboy-disconnected.visible { display: flex; }
        #logboy-disconnected .lb-icon  { font-size: 1.5rem; }
        #logboy-disconnected .lb-title { font-size: 1.1rem; font-weight: bold; color: #f87171; font-family: monospace; }
        #logboy-disconnected .lb-sub   { font-size: 0.85rem; color: #9ca3af; font-family: monospace; }
        </style>
        <div id="logboy-disconnected">
            <div class="lb-icon">⚠</div>
            <div>
                <div class="lb-title">Connection Lost</div>
                <div class="lb-sub">The logboy server is not running.</div>
            </div>
        </div>
        <script>
        (function() {
            const overlay = document.getElementById("logboy-disconnected");
            setInterval(async () => {
                try {
                    await fetch("/", { method: "HEAD", cache: "no-store" });
                    overlay.classList.remove("visible");
                } catch {
                    overlay.classList.add("visible");
                }
            }, 2000);
        })();
        </script>
        ''')
        LogboyGUI(controller, config)

    ui.run(title='logboy', favicon=f'{assets_dir}/logboy_logo.png', reload=False)


if __name__ == '__main__':
    main()