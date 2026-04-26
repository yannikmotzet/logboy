from nicegui import ui, app
import os
import shutil
import subprocess
import yaml
import argparse
from datetime import datetime
from logboy.logboy_controller import LogboyController, list_recordings
from logboy.logboy_stats import TopicSnapshot


def _fmt_bytes(total: float) -> str:
    for unit in ('B', 'KB', 'MB', 'GB'):
        if total < 1024:
            return f'{total:.1f} {unit}'
        total /= 1024
    return f'{total:.1f} TB'


def _build_nav_drawer(active: str = '/') -> None:
    is_expanded = app.storage.user.get('nav_expanded', True)
    state = {'expanded': is_expanded}

    with ui.left_drawer(value=True).props(f'bordered width={"180" if is_expanded else "56"}') as drawer:

        # Collapsed view — icons only
        with ui.column().classes('items-center gap-2 pt-2') as col_mini:
            ui.button(icon='menu', on_click=lambda: toggle()).props('flat round dense')
            ui.separator().classes('w-full my-0')
            ui.button(icon='fiber_manual_record', on_click=lambda: ui.navigate.to('/record')) \
                .props(f'flat round dense {"color=primary" if active == "/record" else ""}')
            ui.button(icon='video_library', on_click=lambda: ui.navigate.to('/storage')) \
                .props(f'flat round dense {"color=primary" if active == "/storage" else ""}')
        col_mini.set_visibility(not is_expanded)

        # Expanded view — icon + label
        with ui.column().classes('gap-0 pt-2 w-full') as col_full:
            with ui.row().classes('px-2 pb-1'):
                ui.button(icon='menu', on_click=lambda: toggle()).props('flat round dense')
            ui.separator().classes('my-1')
            ui.button('Record', icon='fiber_manual_record', on_click=lambda: ui.navigate.to('/record')) \
                .props(f'flat align=left {"color=primary" if active == "/record" else ""}').classes('w-full')
            ui.button('Storage', icon='video_library', on_click=lambda: ui.navigate.to('/storage')) \
                .props(f'flat align=left {"color=primary" if active == "/storage" else ""}').classes('w-full')
        col_full.set_visibility(is_expanded)

    def toggle():
        state['expanded'] = not state['expanded']
        app.storage.user['nav_expanded'] = state['expanded']
        col_mini.set_visibility(not state['expanded'])
        col_full.set_visibility(state['expanded'])
        drawer.props(f'width={"180" if state["expanded"] else "56"}')


def _git_version(start_path: str) -> str:
    try:
        parts = os.path.abspath(start_path).split(os.sep)
        if 'install' in parts:
            ws_root = os.sep.join(parts[:parts.index('install')])
            repo_path = os.path.join(ws_root, 'src', 'logboy')
        else:
            repo_path = os.path.dirname(os.path.abspath(start_path))
        return subprocess.check_output(
            ['git', 'describe', '--tags', '--always', '--dirty'],
            cwd=repo_path,
            stderr=subprocess.DEVNULL,
        ).decode().strip()
    except Exception:
        return 'unknown'


class LogboyGUI:
    def __init__(self, controller: LogboyController, config: dict, config_path: str):
        self.controller  = controller
        self.config      = config
        self.config_path = config_path
        self.is_paused   = False

        self._blink_state = False
        self._ui_recording = False
        self._ui_paused    = False
        self._countdown_remaining = 0
        self._countdown_timer = None
        self._build_ui()

    # ── Build ────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Top bar — wraps to second line only when window is too narrow
        with ui.row().classes('items-center w-full px-4 py-2 flex-wrap gap-y-1'):
            # Left: logo + title (shrink-0 so it never compresses)
            with ui.row().classes('items-center gap-2 shrink-0'):
                ui.image('/assets/logboy_logo.png').classes('w-7 h-7')
                ui.label('logboy').classes('text-lg font-bold')

            ui.element('div').classes('flex-1')

            # Center: buttons + fixed-width status (w-36 keeps width constant)
            with ui.row().classes('items-center gap-1 shrink-0'):
                self.record_btn = ui.button(icon='fiber_manual_record', on_click=self.start_recording) \
                    .props('round flat color="red"')
                self.stop_btn = ui.button(icon='stop', on_click=self.stop_recording) \
                    .props('round flat color="grey-6"')
                self.stop_btn.set_visibility(False)
                self.pause_btn = ui.button(icon='pause', on_click=self.toggle_pause) \
                    .props('round flat color="grey" disable')
                self.rec_indicator = ui.icon('fiber_manual_record', size='xs').classes('text-transparent ml-2')
                self.status_label = ui.label('Ready').classes('text-sm w-36')

            ui.element('div').classes('flex-1')

            # Right: settings + clock + dark (wraps to next line when no space)
            with ui.row().classes('items-center gap-2 shrink-0'):
                ui.label('Max').classes('text-xs text-gray-400')
                self.max_input = ui.input(placeholder='——:——:——') \
                    .props('mask="##:##:##" fill-mask="0" dense borderless hide-bottom-space') \
                    .classes('w-[5.5rem] text-xs font-mono text-gray-400') \
                    .on('blur', self._on_max_blur)
                with self.max_input.add_slot('append'):
                    self.max_clear_btn = ui.icon('close', size='xs') \
                        .classes('cursor-pointer text-gray-400') \
                        .on('click', self._clear_max)
                    self.max_clear_btn.set_visibility(False)
                ui.label('Delay').classes('text-xs text-gray-400 ml-2')
                self.delay_input = ui.input(placeholder='——:——:——') \
                    .props('mask="##:##:##" fill-mask="0" dense borderless hide-bottom-space') \
                    .classes('w-[5.5rem] text-xs font-mono text-gray-400') \
                    .on('blur', self._on_delay_blur)
                with self.delay_input.add_slot('append'):
                    self.delay_clear_btn = ui.icon('close', size='xs') \
                        .classes('cursor-pointer text-gray-400') \
                        .on('click', self._clear_delay)
                    self.delay_clear_btn.set_visibility(False)
                ui.label('Split').classes('text-xs text-gray-400 ml-2')
                self.split_select = ui.select(
                    {'time': 'Time', 'size': 'Size'},
                    value=None, on_change=self._on_split_change,
                ).props('dense borderless hide-bottom-space options-dense label="—"').classes('text-xs font-mono text-gray-400 w-16')
                self.split_time_input = ui.input(placeholder='——:——:——') \
                    .props('mask="##:##:##" fill-mask="0" dense borderless hide-bottom-space') \
                    .classes('w-[5.5rem] text-xs font-mono text-gray-400')
                self.split_time_input.set_visibility(False)
                self.split_size_input = ui.input(placeholder='—') \
                    .props('dense borderless hide-bottom-space suffix="MB"') \
                    .classes('w-14 text-xs font-mono text-gray-400')
                self.split_size_input.set_visibility(False)
                self.split_clear_btn = ui.icon('close', size='xs') \
                    .classes('cursor-pointer text-gray-400') \
                    .on('click', self._clear_split)
                self.split_clear_btn.set_visibility(False)
                self.clock_label = ui.label().classes('text-xs font-mono text-gray-400 ml-2')
                self.dark = ui.dark_mode(value=app.storage.user.get('dark_mode', True))
                self.dark_btn = ui.button(icon='dark_mode' if app.storage.user.get('dark_mode', True) else 'light_mode',
                                          on_click=self._toggle_dark).props('flat round dense')

        # pre-populate max_duration from config if set
        cfg_max = self.config.get('max_duration')
        if cfg_max:
            h, m, s = self._secs_to_hms(cfg_max)
            self.max_input.value = f'{h:02d}:{m:02d}:{s:02d}'

        # Recording info panel
        with ui.row().classes('items-center justify-center w-full gap-8 px-4 py-1') as self.rec_info_row:
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
        ui.separator().classes('my-2')
        with ui.row().classes('items-center px-4 pb-1 gap-2'):
            ui.label('Topics').classes('text-sm font-semibold')
            self.topics_count_label = ui.label('').classes('text-xs text-gray-400')
            ui.element('div').classes('flex-1')
            self.topics_btn = ui.button(icon='tune', on_click=self._open_topics_dialog) \
                .props('flat round dense').tooltip('Select topics')
        columns = [
            {'name': 'name',         'label': 'Topic',    'field': 'name',         'align': 'left',  'sortable': True},
            {'name': 'expected_fps', 'label': 'Exp. FPS', 'field': 'expected_fps', 'align': 'right', 'sortable': True},
            {'name': 'fps',          'label': 'FPS',      'field': 'fps',          'align': 'right', 'sortable': True},
            {'name': 'drops',        'label': 'Drops',    'field': 'drops',        'align': 'right', 'sortable': True},
            {'name': 'total_msgs',   'label': 'Msgs',     'field': 'total_msgs',   'align': 'right', 'sortable': True},
            {'name': 'age',          'label': 'Age (s)',  'field': 'age',          'align': 'right', 'sortable': True},
        ]
        self.table = ui.table(columns=columns, rows=[], row_key='name').classes('w-full')

        self.table.add_slot('body-cell-name', '''
            <q-td :props="props">
                <span :style="{ color:
                    props.row.expected_fps <= 0                                                          ? 'inherit' :
                    !props.row.active || props.row.age > 5                                               ? '#ef4444' :
                    props.row.age > 1/props.row.expected_fps*3                                           ? '#eab308' : 'inherit' }">
                    {{ props.value }}
                </span>
            </q-td>
        ''')
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
                <span :style="{ color: (() => {
                    const rate = props.row.total_msgs > 0 ? props.value / props.row.total_msgs : 0;
                    return rate > 0.05 ? '#ef4444' : rate > 0 ? '#eab308' : '#22c55e';
                })() }">
                    {{ props.value }}
                    <span v-if="props.row.total_msgs > 0" style="opacity:0.6; font-size:0.85em">
                        ({{ (props.value / props.row.total_msgs * 100).toFixed(1) }}%)
                    </span>
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
        self.blink_timer       = ui.timer(1.0,   self._blink_record,   active=False)
        self.blink_pause_timer = ui.timer(0.25,  self._blink_pause,    active=False)
        self.elapsed_timer     = ui.timer(0.1,   self._refresh_elapsed, active=False)
        self.monitor_timer     = ui.timer(1.0,   self._refresh_table,  active=True)

    # ── Transport controls ───────────────────────────────────────────────────

    def start_recording(self):
        if self._countdown_timer is not None:
            self._cancel_countdown()
            return
        delay = self._get_delay_secs()
        if delay > 0:
            self._set_enabled(self.max_input, False); self._set_enabled(self.delay_input, False); self._set_enabled(self.split_select, False); self._set_enabled(self.split_time_input, False); self._set_enabled(self.split_size_input, False)
            self._countdown_remaining = int(delay)
            self._update_countdown_label()
            self._countdown_timer = ui.timer(1.0, self._countdown_tick)
        else:
            self._do_start_recording()

    def _cancel_countdown(self):
        self._countdown_timer.cancel()
        self._countdown_timer = None
        self._countdown_remaining = 0
        self.status_label.set_text('Ready')
        self._set_enabled(self.max_input, True); self._set_enabled(self.delay_input, True); self._set_enabled(self.split_select, True); self._set_enabled(self.split_time_input, True); self._set_enabled(self.split_size_input, True)

    def _countdown_tick(self):
        self._countdown_remaining -= 1
        if self._countdown_remaining <= 0:
            self._countdown_timer.cancel()
            self._countdown_timer = None
            self._do_start_recording()
        else:
            self._update_countdown_label()

    def _update_countdown_label(self):
        self.status_label.set_text(f'Starting in {self._countdown_remaining}s…')

    def _do_start_recording(self):
        self.controller.set_max_duration(self._get_max_duration_secs())
        try:
            self.controller.start_recording()
        except ValueError as e:
            ui.notify(str(e), type='negative', position='top', timeout=5000)
            self._set_enabled(self.max_input, True); self._set_enabled(self.delay_input, True); self._set_enabled(self.split_select, True); self._set_enabled(self.split_time_input, True); self._set_enabled(self.split_size_input, True)
            return
        self.is_paused = False
        self.status_label.set_text('Recording')
        self.record_btn.set_visibility(False)
        self.stop_btn.set_visibility(True)
        self._set_enabled(self.pause_btn, True)
        self._set_enabled(self.max_input, False); self._set_enabled(self.delay_input, False); self._set_enabled(self.split_select, False); self._set_enabled(self.split_time_input, False); self._set_enabled(self.split_size_input, False)
        self.rec_indicator.classes('text-red-500', remove='text-transparent')
        self._set_enabled(self.topics_btn, False)
        self.blink_timer.activate()
        self.elapsed_timer.activate()

    def stop_recording(self):
        self.controller.stop_recording()
        self.is_paused = False
        self.status_label.set_text('Stopped')
        self.blink_timer.deactivate()
        self.blink_pause_timer.deactivate()
        self.elapsed_timer.deactivate()
        self.pause_btn.props('icon=pause color=grey')
        self.stop_btn.set_visibility(False)
        self.record_btn.set_visibility(True)
        self._set_enabled(self.record_btn, True)
        self._set_enabled(self.pause_btn, False)
        self._set_enabled(self.max_input, True); self._set_enabled(self.delay_input, True); self._set_enabled(self.split_select, True); self._set_enabled(self.split_time_input, True); self._set_enabled(self.split_size_input, True)
        self.rec_indicator.props('name=fiber_manual_record')
        self.rec_indicator.classes('text-transparent', remove='text-red-500 text-orange-500')
        self._set_enabled(self.topics_btn, True)

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
        configured = {t['name'] for t in (self.config.get('topics') or [])}
        snapshots: list[TopicSnapshot] = [s for s in self.controller.get_stats() if s.name in configured]
        self.table.rows = [self._snapshot_to_row(s) for s in sorted(snapshots, key=lambda s: s.name)]
        self.table.update()
        n = len(self.config.get('topics') or [])
        self.topics_count_label.set_text(f'{n} configured')
        self.controller.check_max_duration()
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
            self.elapsed_timer.deactivate()
            self.pause_btn.props('icon=pause color=grey')
            self._set_enabled(self.max_input, True); self._set_enabled(self.delay_input, True); self._set_enabled(self.split_select, True); self._set_enabled(self.split_time_input, True); self._set_enabled(self.split_size_input, True)
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
            self._set_enabled(self.max_input, False); self._set_enabled(self.delay_input, False); self._set_enabled(self.split_select, False); self._set_enabled(self.split_time_input, False); self._set_enabled(self.split_size_input, False)
            self.rec_indicator.classes('text-red-500', remove='text-transparent text-orange-500')
            self.blink_timer.activate()
            self.elapsed_timer.activate()

        self._ui_recording = is_recording
        self._ui_paused    = is_paused
        self.is_paused     = is_paused

    # ── Topic selection ──────────────────────────────────────────────────────

    def _open_topics_dialog(self):
        config_topics  = {t['name']: t for t in (self.config.get('topics') or [])}
        all_topics_map = {}
        checkboxes     = {}
        fps_inputs     = {}
        fps_labels     = {}
        saved_state    = {}  # preserves user edits across refreshes

        def discover():
            active_topics = self.controller.node.discover_topics()
            active_names  = {t['name'] for t in active_topics}
            new_map = {t['name']: t for t in active_topics}
            new_map.update(config_topics)
            all_topics_map.clear()
            all_topics_map.update(new_map)
            return active_names

        active_names = discover()

        with ui.dialog().props('persistent') as dialog, ui.card().classes('q-pa-sm').style('width: 1400px; max-width: 90vw'):
            with ui.row().classes('items-center w-full mb-1'):
                ui.label('Select Topics').classes('text-base font-semibold')
                ui.element('div').classes('flex-1')
                ui.button('All',  on_click=lambda: [cb.set_value(True)  for cb in checkboxes.values()]).props('flat dense size=sm')
                ui.button('None', on_click=lambda: [cb.set_value(False) for cb in checkboxes.values()]).props('flat dense size=sm')
                ui.button(icon='refresh', on_click=lambda: refresh_list()).props('flat round dense').tooltip('Re-discover topics')

            search = ui.input(placeholder='Filter topics…') \
                .props('dense clearable outlined').classes('w-full mb-1')

            with ui.row().classes('w-full text-xs text-gray-400 px-2 gap-2'):
                ui.element('div').classes('w-6')
                ui.label('Topic').classes('flex-1 min-w-[8rem] font-mono')
                ui.label('Type').classes('w-72 min-w-0 font-mono')
                ui.label('Exp FPS').classes('w-20 text-right shrink-0')
                ui.label('Live FPS').classes('w-20 text-right shrink-0')

            ui.separator()

            rows = {}  # name → row element

            @ui.refreshable
            def topic_list():
                all_topics = sorted(all_topics_map.values(), key=lambda t: t['name'])
                checkboxes.clear()
                fps_inputs.clear()
                fps_labels.clear()
                rows.clear()
                for t in all_topics:
                    name    = t['name']
                    prev    = saved_state.get(name, {})
                    checked = prev.get('checked', name in config_topics)
                    fps_val = prev.get('fps', t['fps'] if t.get('fps') not in (None, 0, 0.0) else None)
                    muted   = 'text-gray-400' if name not in active_names else ''
                    with ui.row().classes('items-center w-full px-2 gap-2').style('min-height:0; height:1.4rem') as row:
                        cb = ui.checkbox(value=checked).props('dense')
                        checkboxes[name] = cb
                        ui.label(name).classes(f'flex-1 min-w-[8rem] truncate text-sm font-mono {muted}')
                        ui.label(t.get('type', '')).classes('w-72 min-w-0 truncate text-xs font-mono text-gray-400')
                        inp = ui.number(value=fps_val, min=0, step=0.1, placeholder='—') \
                            .props('dense borderless hide-bottom-space').classes('w-20 text-right text-xs shrink-0')
                        fps_inputs[name] = inp
                        fps_labels[name] = ui.label('—').classes('w-20 text-right text-xs shrink-0')
                    rows[name] = row

            search.on('update:model-value', lambda e: [
                rows[name].set_visibility((e.args or '').lower() in name.lower())
                for name in rows
            ])

            with ui.scroll_area().classes('h-[32rem] w-full'):
                topic_list()

            def refresh_list():
                for name in list(checkboxes):
                    saved_state[name] = {'checked': checkboxes[name].value, 'fps': fps_inputs[name].value}
                nonlocal active_names
                active_names = discover()
                topic_list.refresh()

            def refresh_fps():
                stats = {s.name: s for s in self.controller.get_stats()}
                for name, lbl in fps_labels.items():
                    s = stats.get(name)
                    if s and s.first_seen:
                        lbl.set_text(f'{s.fps:.1f}')
                        if fps_inputs[name].value is None:
                            fps_inputs[name].set_value(round(s.fps, 1))
                    else:
                        lbl.set_text('—')

            fps_timer = ui.timer(0.5, refresh_fps)

            with ui.row().classes('justify-end gap-2 mt-3 w-full'):
                ui.button('Cancel', on_click=lambda: (fps_timer.cancel(), dialog.close())).props('flat')
                ui.button('Save',   on_click=lambda: self._save_topics(
                    dialog, fps_timer, checkboxes, fps_inputs, all_topics_map,
                )).props('color=primary')

        dialog.open()

    def _save_topics(self, dialog, fps_timer, checkboxes, fps_inputs, all_topics_map):
        fps_timer.cancel()

        new_topics = []
        for name, cb in checkboxes.items():
            if not cb.value:
                continue
            t = dict(all_topics_map[name])
            fps_val = fps_inputs[name].value
            t['fps'] = float(fps_val) if fps_val else 0
            new_topics.append(t)

        self.config['topics'] = new_topics
        with open(self.config_path, 'w') as f:
            yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)
        self.controller.configure_recorder(self.config)

        dialog.close()
        ui.notify(f'{len(new_topics)} topic(s) saved.', type='positive', position='top')

    def _refresh_elapsed(self):
        elapsed = self.controller.get_elapsed()
        if elapsed is None:
            return
        h, rem = divmod(int(elapsed), 3600)
        m, s = divmod(rem, 60)
        tenths = int(elapsed * 10) % 10
        self.elapsed_label.set_text(f'{h:02d}:{m:02d}:{s:02d}.{tenths}')

    def _refresh_rec_info(self):
        elapsed = self.controller.get_elapsed()
        if elapsed is None:
            return

        max_dur = self.controller.get_max_duration()
        bag_path = self.controller.get_bag_path()
        if max_dur and not self.is_paused:
            remaining = max(0.0, max_dur - elapsed)
            rh, rrem = divmod(int(remaining), 3600)
            rm, rs = divmod(rrem, 60)
            self.status_label.set_text(f'Recording · {rh:02d}:{rm:02d}:{rs:02d} left')
        elif not self.is_paused:
            self.status_label.set_text('Recording')
        if bag_path:
            self.rec_name_label.set_text(os.path.basename(bag_path))
            self.rec_path_label.set_text(os.path.dirname(bag_path))
            size_bytes = self._bag_size_bytes(bag_path)
            self.size_label.set_text(_fmt_bytes(size_bytes))
            # update time-left estimate in free label
            if elapsed > 5 and size_bytes > 0:
                storage_path = self.config.get('storage_path', '')
                try:
                    free_bytes = shutil.disk_usage(os.path.expanduser(storage_path)).free
                    rate = size_bytes / elapsed  # bytes/s
                    secs_left = int(free_bytes / rate)
                    h2, r2 = divmod(secs_left, 3600)
                    m2 = r2 // 60
                    self.free_label.set_text(f'{_fmt_bytes(free_bytes)} ~{h2}h{m2:02d}m')
                except OSError:
                    pass

    def _refresh_free_space(self):
        """Update free space label when not recording (no rate estimate available)."""
        if self.controller.get_elapsed() is not None:
            return  # handled by _refresh_rec_info
        storage_path = os.path.expanduser(self.config.get('storage_path', ''))
        if not os.path.isdir(storage_path):
            self.rec_path_label.classes('text-red-500', remove='text-gray-400')
            self.free_label.set_text('—')
            self.free_label.classes('text-gray-400', remove='text-red-500')
            return
        self.rec_path_label.classes('text-gray-400', remove='text-red-500')
        self.free_label.classes('text-gray-400', remove='text-red-500')
        try:
            free_bytes = shutil.disk_usage(storage_path).free
            self.free_label.set_text(_fmt_bytes(free_bytes))
        except OSError:
            self.free_label.set_text('—')

    @staticmethod
    def _bag_size_bytes(bag_path: str) -> int:
        if not os.path.isdir(bag_path):
            return 0
        return sum(e.stat().st_size for e in os.scandir(bag_path) if e.is_file())

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

    def _on_split_change(self, *_):
        mode = self.split_select.value
        self.split_time_input.set_visibility(mode == 'time')
        self.split_size_input.set_visibility(mode == 'size')
        self.split_clear_btn.set_visibility(mode is not None)
        if mode is None:
            self.split_time_input.value = ''
            self.split_size_input.value = ''
            self.split_select.props('label="—"')
        else:
            self.split_select.props(remove='label')
            ui.notify('Split recording is not yet supported — backend coming soon.', type='warning', position='top', timeout=3000)

    def _clear_split(self, *_):
        self.split_select.value = None
        self.split_time_input.value = ''
        self.split_size_input.value = ''
        self.split_time_input.set_visibility(False)
        self.split_size_input.set_visibility(False)
        self.split_clear_btn.set_visibility(False)
        self.split_select.props('label="—"')

    def get_split_config(self) -> dict:
        """Returns {'mode': 'off'|'time'|'size', 'value': seconds|bytes|None}"""
        mode = self.split_select.value
        if mode is None:
            return {'mode': 'off', 'value': None}
        if mode == 'time':
            secs = self._parse_hms(self.split_time_input.value)
            return {'mode': 'time', 'value': secs or None}
        if mode == 'size':
            try:
                mb = float(self.split_size_input.value)
                return {'mode': 'size', 'value': int(mb * 1024 * 1024) if mb else None}
            except (ValueError, TypeError):
                return {'mode': 'size', 'value': None}
        return {'mode': 'off', 'value': None}

    def _on_max_blur(self, *_):
        if not self._get_max_duration_secs():
            self.max_input.value = ''
        self.max_clear_btn.set_visibility(bool(self._get_max_duration_secs()))

    def _on_delay_blur(self, *_):
        if not self._get_delay_secs():
            self.delay_input.value = ''
        self.delay_clear_btn.set_visibility(bool(self._get_delay_secs()))

    def _clear_max(self, *_):
        self.max_input.value = ''
        self.max_clear_btn.set_visibility(False)

    def _clear_delay(self, *_):
        self.delay_input.value = ''
        self.delay_clear_btn.set_visibility(False)

    def _get_max_duration_secs(self) -> float | None:
        total = self._parse_hms(self.max_input.value)
        return float(total) if total > 0 else None

    def _get_delay_secs(self) -> float:
        return float(self._parse_hms(self.delay_input.value))

    @staticmethod
    def _parse_hms(value: str) -> int:
        parts = (value or '').split(':')
        if len(parts) != 3:
            return 0
        try:
            return int(parts[0] or 0) * 3600 + int(parts[1] or 0) * 60 + int(parts[2] or 0)
        except ValueError:
            return 0

    @staticmethod
    def _secs_to_hms(secs: float) -> tuple[int, int, int]:
        secs = int(secs)
        h, rem = divmod(secs, 3600)
        m, s = divmod(rem, 60)
        return h, m, s

    def _toggle_dark(self):
        self.dark.toggle()
        app.storage.user['dark_mode'] = self.dark.value
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


# ── Recordings page ──────────────────────────────────────────────────────────

class RecordingsPage:
    def __init__(self, controller: LogboyController, config: dict):
        self.controller = controller
        self.config = config
        self._build_ui()

    def _build_ui(self):
        with ui.row().classes('items-center w-full px-4 py-2 gap-2'):
            ui.image('/assets/logboy_logo.png').classes('w-7 h-7')
            ui.label('logboy').classes('text-lg font-bold')
            ui.label('· Recordings').classes('text-lg text-gray-400')
            ui.label(os.path.expanduser(self.config.get('storage_path', '—'))) \
                .classes('text-xs font-mono text-gray-400 ml-2')
            ui.element('div').classes('flex-1')
            self.count_label = ui.label().classes('text-xs text-gray-400')
            ui.button(icon='refresh', on_click=self._load).props('flat round dense').tooltip('Refresh')
            dark = ui.dark_mode(value=app.storage.user.get('dark_mode', True))
            dark_btn = ui.button(
                icon='dark_mode' if app.storage.user.get('dark_mode', True) else 'light_mode',
                on_click=lambda: (dark.toggle(),
                                  app.storage.user.update({'dark_mode': dark.value}),
                                  dark_btn.props('icon=light_mode' if dark.value else 'icon=dark_mode')),
            ).props('flat round dense')

        ui.separator().classes('my-2')

        columns = [
            {'name': 'name',     'label': 'Name',     'field': 'name',     'align': 'left',  'sortable': True},
            {'name': 'robot',    'label': 'Robot',    'field': 'robot',    'align': 'left',  'sortable': True},
            {'name': 'date',     'label': 'Date',     'field': 'date',     'align': 'right', 'sortable': True},
            {'name': 'duration', 'label': 'Duration', 'field': 'duration', 'align': 'right', 'sortable': False},
            {'name': 'size',     'label': 'Size',     'field': 'size',     'align': 'right', 'sortable': False},
            {'name': 'topics',   'label': 'Topics',   'field': 'topics',   'align': 'right', 'sortable': True},
            {'name': 'messages', 'label': 'Messages', 'field': 'messages', 'align': 'right', 'sortable': True},
            {'name': 'actions',  'label': '',         'field': 'actions',  'align': 'right', 'sortable': False},
        ]
        self.table = ui.table(columns=columns, rows=[], row_key='name').classes('w-full')

        self.table.add_slot('body-cell-name', '''
            <q-td :props="props">
                <q-icon v-if="props.row.is_active" name="fiber_manual_record"
                        size="xs" class="text-red-500 mr-1" />
                {{ props.value }}
            </q-td>
        ''')
        self.table.add_slot('body-cell-actions', '''
            <q-td :props="props">
                <q-btn flat round dense icon="delete" color="red"
                       :disable="props.row.is_active"
                       @click="$parent.$emit('delete', props.row)" />
            </q-td>
        ''')
        self.table.on('delete', lambda e: self._confirm_delete(e.args))

        self._load()

    def _load(self):
        storage_path = self.config.get('storage_path', '')
        active_path  = self.controller.get_bag_path()
        recs = list_recordings(storage_path, compute_md5=False)

        rows = []
        for r in recs:
            date_str = datetime.fromtimestamp(r.start_time).strftime('%Y-%m-%d %H:%M:%S') if r.start_time else '—'
            if r.duration is not None:
                h, rem = divmod(int(r.duration), 3600)
                m, s   = divmod(rem, 60)
                dur_str = f'{h:02d}:{m:02d}:{s:02d}' if h else f'{m:02d}:{s:02d}'
            else:
                dur_str = '—'
            total_msgs  = sum(t.get('count', 0) for t in r.topics)
            is_active   = active_path is not None and \
                          os.path.normpath(r.path) == os.path.normpath(active_path)
            rows.append({
                'name':      r.name,
                'robot':     r.robot or '—',
                'date':      date_str,
                'duration':  dur_str,
                'size':      _fmt_bytes(r.size),
                'topics':    len(r.topics),
                'messages':  total_msgs,
                'is_active': is_active,
                'path':      r.path,
            })

        self.table.rows = rows
        self.table.update()
        self.count_label.set_text(f'{len(recs)} recording(s)')

    def _confirm_delete(self, row: dict):
        path = row['path']
        name = row['name']
        with ui.dialog() as dialog, ui.card():
            ui.label(f'Delete "{name}"?').classes('text-base font-semibold')
            ui.label('This permanently removes the recording directory.') \
                .classes('text-sm text-gray-400 mt-1')
            with ui.row().classes('justify-end gap-2 mt-4'):
                ui.button('Cancel', on_click=dialog.close).props('flat')
                ui.button('Delete', on_click=lambda: (
                    shutil.rmtree(path),
                    dialog.close(),
                    self._load(),
                )).props('color=red')
        dialog.open()


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Logboy GUI')
    parser.add_argument('--config', required=True, help='Path to YAML config file')
    args = parser.parse_args()

    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)

    controller = LogboyController()
    controller.configure_recorder(config)   # ← einmalig hier, nicht in start_recording
    version = _git_version(args.config)

    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
    assets_dir = os.path.join(ament_prefix_path.split(os.pathsep)[0], 'share', 'logboy', 'assets')
    app.add_static_files('/assets', assets_dir)

    @ui.page('/')
    def root():
        ui.navigate.to('/record')

    @ui.page('/record')
    def index():
        _build_nav_drawer(active='/record')
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
        LogboyGUI(controller, config, args.config)
        with ui.element('div').classes('fixed bottom-0 left-0 right-0 flex items-center justify-center gap-2 py-1'):
            ui.link('logboy', 'https://github.com/yannikmotzet/logboy', new_tab=True) \
                .classes('text-xs text-gray-600 hover:text-gray-400 no-underline')
            ui.label('·').classes('text-xs text-gray-600')
            ui.label(version).classes('text-xs font-mono text-gray-600 select-all')

    @ui.page('/storage')
    def storage_page():
        _build_nav_drawer(active='/storage')
        RecordingsPage(controller, config)
        with ui.element('div').classes('fixed bottom-0 left-0 right-0 flex items-center justify-center gap-2 py-1'):
            ui.link('logboy', 'https://github.com/yannikmotzet/logboy', new_tab=True) \
                .classes('text-xs text-gray-600 hover:text-gray-400 no-underline')
            ui.label('·').classes('text-xs text-gray-600')
            ui.label(version).classes('text-xs font-mono text-gray-600 select-all')

    ui.run(title='logboy', favicon=f'{assets_dir}/logboy_logo.png', reload=False, storage_secret='logboy')


if __name__ == '__main__':
    main()