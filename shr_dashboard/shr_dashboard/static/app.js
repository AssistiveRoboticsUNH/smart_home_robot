(() => {
  const state = {
    route: 'home',
    loading: true,
    bootError: null,
    config: null,
    metadata: null,
    yamlPath: '',
    yamlPreview: '',
    selectedProtocol: null,
    flash: null,
    reload: {
      pollTimer: null,
    },
    dirty: false,
    validation: {
      status: 'idle', // idle | ok | error
      messages: [],
      lastValidatedAt: null,
    },
    lastSavedAt: null,
    lastSaveInfo: null,
    ui: {
      listSearch: '',
      listSort: 'name_asc',
      filters: {
        hasTime: false,
        hasEvent: false,
      },
      specialCollapsed: true,
      sidebarCollapsed: false,
      sectionsOpen: {
        general: true,
        triggers: true,
        reset: true,
        actions: true,
        other_params: false,
        yaml: false,
      },
      scroll: {
        list: 0,
        editor: 0,
      },
      stepCollapsed: {},
      recentProtocols: [],
    },
    history: {
      mode: 'single', // single | range | all
      date: new Date().toISOString().slice(0, 10),
      dateFrom: new Date(Date.now() - 6 * 24 * 60 * 60 * 1000).toISOString().slice(0, 10),
      dateTo: new Date().toISOString().slice(0, 10),
      page: 1,
      pageSize: 20,
      statusFilter: '',
      protocolFilter: '',
      entries: [],
      states: [],
      loading: false,
      error: null,
    },
    home: {
      robotState: null,
      protocolStates: [],
      summary: null,
      pollTimer: null,
      error: null,
    },
  };

  const els = {
    appShell: document.getElementById('app'),
    sidebarToggle: document.getElementById('sidebarToggle'),
    routeView: document.getElementById('routeView'),
    pageTitle: document.getElementById('pageTitle'),
    pageSubtitle: document.getElementById('pageSubtitle'),
    statusChip: document.getElementById('statusChip'),
    yamlPathLabel: document.getElementById('yamlPathLabel'),
    flashBar: document.getElementById('flashBar'),
    navItems: [...document.querySelectorAll('.nav-item')],
  };

  const deepClone = (obj) => JSON.parse(JSON.stringify(obj));

  function nowTimeLabel() {
    const d = new Date();
    return d.toLocaleTimeString();
  }

  function setFlash(type, message) {
    state.flash = message ? { type, message } : null;
    renderFlash();
  }

  function clearFlash() {
    state.flash = null;
    renderFlash();
  }

  function renderFlash() {
    if (!state.flash) {
      els.flashBar.className = 'flash hidden';
      els.flashBar.innerHTML = '';
      return;
    }
    els.flashBar.className = `flash ${state.flash.type}`;
    els.flashBar.innerHTML = `
      <div class="flash-message">${escapeHtml(state.flash.message)}</div>
      <button class="flash-close" type="button" data-action="flash-close" aria-label="Close notification" title="Close">×</button>
    `;
  }

  function stopReloadPoll() {
    if (state.reload.pollTimer) {
      window.clearTimeout(state.reload.pollTimer);
      state.reload.pollTimer = null;
    }
  }

  async function pollReloadStatus() {
    try {
      const data = await api('/api/dashboard/reload-status');
      const reload = data.reload || {};
      if (reload.state === 'applied') {
        stopReloadPoll();
        setFlash('success', 'Runtime YAML reloaded.');
        return;
      }
      if (reload.state === 'error') {
        stopReloadPoll();
        setFlash('error', `Runtime YAML reload failed: ${reload.error || 'unknown error'}`);
        return;
      }
      if (reload.state === 'pending') {
        state.reload.pollTimer = window.setTimeout(pollReloadStatus, 1500);
        return;
      }
      stopReloadPoll();
    } catch (_err) {
      state.reload.pollTimer = window.setTimeout(pollReloadStatus, 2000);
    }
  }

  async function api(path, options = {}) {
    const res = await fetch(path, {
      headers: { 'Content-Type': 'application/json' },
      ...options,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok || data.ok === false) {
      throw new Error(data.error || `Request failed (${res.status})`);
    }
    return data;
  }

  function metadataForTree(treeName) {
    return state.metadata?.run_trees?.[treeName] || { params: [], description: '' };
  }

  function robotStateSpecs() {
    return state.metadata?.robot_states || [];
  }

  function robotStateSpec(stateKey) {
    return robotStateSpecs().find((s) => s.name === stateKey) || null;
  }

  function eventUiKindForState(stateKey) {
    return robotStateSpec(stateKey)?.ui_kind || null;
  }

  function eventKindForState(stateKey, fallbackValue) {
    const spec = robotStateSpec(stateKey);
    if (spec?.kind) {
      const k = spec.kind;
      if (k === 'int' || k === 'float' || k === 'number') return 'number';
      return k;
    }
    return guessValueType(fallbackValue);
  }

  function locationValueOptionsForState(stateKey) {
    const locs = Object.keys(state.config?.locations || {});
    if (stateKey === 'position') {
      const set = new Set(locs);
      set.add('away');
      return [...set];
    }
    return locs;
  }

  function executionLocationOptions() {
    return ['current', 'person', ...Object.keys(state.config?.locations || {})];
  }

  function personInitOptions() {
    return ['', ...Object.keys(state.config?.locations || {})];
  }

  function protocolTriggerOptions(currentValue = '') {
    const options = Object.entries(getProtocols()).map(([name, protocol]) => {
      const runner = protocol?.runner || 'GenericProtocol';
      return `${runner}.${name}`;
    }).sort();
    const current = String(currentValue || '').trim();
    if (current && !options.includes(current)) options.unshift(current);
    return options;
  }

  function assetOptions(kind, currentValue = '') {
    const assets = state.metadata?.user_assets || {};
    const configured = kind === 'audio_asset' ? (assets.audio_files || []) : (assets.video_files || []);
    const options = [...configured];
    const current = String(currentValue || '').trim();
    if (current && !options.includes(current)) options.unshift(current);
    if (!options.includes('')) options.unshift('');
    return options;
  }

  function guessValueType(value) {
    if (typeof value === 'boolean') return 'bool';
    if (typeof value === 'number') return 'number';
    return 'string';
  }

  function makeDefaultEventTrigger() {
    return { state: '', value: '' };
  }

  function makeDefaultProtocolCompletionTrigger() {
    return {
      protocol: '',
      statuses: ['completed'],
      within_seconds: 120,
    };
  }

  function makeDefaultTriggerNode(kind = 'event') {
    if (kind === 'event') return { event: makeDefaultEventTrigger() };
    if (kind === 'time') return { time: { from: '', to: '', day: [] } };
    if (kind === 'permissible_locations') return { permissible_locations: [] };
    if (kind === 'protocol_completion') return { protocol_completion: makeDefaultProtocolCompletionTrigger() };
    if (kind === 'all') return { all: [] };
    if (kind === 'any') return { any: [] };
    return { event: makeDefaultEventTrigger() };
  }

  function triggerNodeMode(node) {
    if (node && typeof node === 'object' && !Array.isArray(node)) {
      if (Array.isArray(node.all)) return 'all';
      if (Array.isArray(node.any)) return 'any';
    }
    return '';
  }

  function normalizeEventRule(evt) {
    const next = deepClone(evt || makeDefaultEventTrigger());
    normalizeScalarEvent(next);
    return next;
  }

  function normalizeProtocolCompletionRule(rule) {
    const next = deepClone(rule || makeDefaultProtocolCompletionTrigger());
    next.protocol = String(next.protocol || '').trim();
    next.statuses = Array.isArray(next.statuses) ? next.statuses.filter(Boolean) : ['completed'];
    if (!next.statuses.length) next.statuses = ['completed'];
    next.within_seconds = Number(next.within_seconds || 120);
    if (!Number.isFinite(next.within_seconds) || next.within_seconds <= 0) next.within_seconds = 120;
    if (next.after_seconds === '' || next.after_seconds == null) delete next.after_seconds;
    else {
      next.after_seconds = Number(next.after_seconds);
      if (!Number.isFinite(next.after_seconds) || next.after_seconds < 0) delete next.after_seconds;
    }
    return next;
  }

  function normalizeTimeRule(time) {
    const next = deepClone(time || {});
    next.day = Array.isArray(next.day) ? next.day : [];
    if (next.at != null && next.at !== '') {
      const exact = String(next.at);
      delete next.at;
      next.from = exact;
      next.to = exact;
    } else {
      next.from = next.from == null ? '' : String(next.from);
      next.to = next.to == null ? '' : String(next.to);
    }
    delete next.at;
    return next;
  }

  function splitTimeValue(value) {
    const raw = String(value || '');
    const match = /^([01]\d|2[0-3]):([0-5]\d)$/.exec(raw);
    if (!match) return { hour: '', minute: '' };
    return { hour: match[1], minute: match[2] };
  }

  function splitTimeValue12(value) {
    const raw = String(value || '');
    const match = /^([01]\d|2[0-3]):([0-5]\d)$/.exec(raw);
    if (!match) return { hour: '', minute: '', meridiem: '' };
    let hour24 = Number(match[1]);
    const minute = match[2];
    const meridiem = hour24 >= 12 ? 'PM' : 'AM';
    hour24 %= 12;
    if (hour24 === 0) hour24 = 12;
    return {
      hour: String(hour24).padStart(2, '0'),
      minute,
      meridiem,
    };
  }

  function to24HourTime(hour12, minute, meridiem) {
    const h12 = Number(hour12);
    if (!Number.isInteger(h12) || h12 < 1 || h12 > 12) return '';
    if (!/^\d{2}$/.test(String(minute))) return '';
    if (!['AM', 'PM'].includes(String(meridiem))) return '';
    let hour24 = h12 % 12;
    if (meridiem === 'PM') hour24 += 12;
    return `${String(hour24).padStart(2, '0')}:${String(minute)}`;
  }

  function setTimePart(currentValue, part, nextPiece) {
    const current = splitTimeValue12(currentValue);
    const nextHour = part === 'hour' ? nextPiece : (current.hour || '12');
    const nextMinute = part === 'minute' ? nextPiece : (current.minute || '00');
    const nextMeridiem = part === 'meridiem' ? nextPiece : (current.meridiem || 'AM');
    return to24HourTime(nextHour, nextMinute, nextMeridiem);
  }

  function renderTimeSelect(field, value, path) {
    const parts = splitTimeValue12(value);
    const hours = Array.from({ length: 12 }, (_, idx) => String(idx + 1).padStart(2, '0'));
    const minutes = Array.from({ length: 60 }, (_, idx) => String(idx).padStart(2, '0'));
    return `
      <div class="time-clock">
        <div class="time-select-row">
          <select data-action="trigger-time-part" data-path="${escapeAttr(path)}" data-field="${field}" data-part="hour">
            <option value="">HH</option>
          ${hours.map((h) => `<option value="${h}" ${parts.hour === h ? 'selected' : ''}>${h}</option>`).join('')}
          </select>
          <span class="time-sep">:</span>
          <select data-action="trigger-time-part" data-path="${escapeAttr(path)}" data-field="${field}" data-part="minute">
            <option value="">MM</option>
          ${minutes.map((m) => `<option value="${m}" ${parts.minute === m ? 'selected' : ''}>${m}</option>`).join('')}
          </select>
        </div>
        <div class="ampm-toggle" role="group" aria-label="${escapeAttr(`${field} meridiem`)}">
          <button type="button" class="ampm-btn ${parts.meridiem === 'AM' ? 'active' : ''}" data-action="trigger-time-meridiem" data-path="${escapeAttr(path)}" data-field="${field}" data-value="AM">AM</button>
          <button type="button" class="ampm-btn ${parts.meridiem === 'PM' ? 'active' : ''}" data-action="trigger-time-meridiem" data-path="${escapeAttr(path)}" data-field="${field}" data-value="PM">PM</button>
        </div>
      </div>`;
  }

  function normalizeTriggerNode(node, forceRoot = false) {
    if (!node || typeof node !== 'object' || Array.isArray(node)) {
      return forceRoot ? {} : makeDefaultTriggerNode('event');
    }

    const keys = Object.keys(node).filter((key) => node[key] !== undefined);
    if (!keys.length) return forceRoot ? {} : makeDefaultTriggerNode('event');

    if (keys.length === 1) {
      const only = keys[0];
      if (only === 'all' || only === 'any') {
        return {
          [only]: Array.isArray(node[only])
            ? node[only].map((child) => normalizeTriggerNode(child, false))
            : [],
        };
      }
      if (only === 'event') {
        const rules = Array.isArray(node.event) ? node.event : [node.event];
        const normalizedRules = rules.filter(Boolean).map((evt) => normalizeEventRule(evt));
        if (!normalizedRules.length) return { event: makeDefaultEventTrigger() };
        if (normalizedRules.length === 1) {
          const single = { event: normalizedRules[0] };
          return forceRoot ? { all: [single] } : single;
        }
        return { all: normalizedRules.map((evt) => ({ event: evt })) };
      }
      if (only === 'time') {
        const single = { time: normalizeTimeRule(node.time) };
        return forceRoot ? { all: [single] } : single;
      }
      if (only === 'protocol_completion') {
        const single = { protocol_completion: normalizeProtocolCompletionRule(node.protocol_completion) };
        return forceRoot ? { all: [single] } : single;
      }
      if (only === 'permissible_locations') {
        const single = {
          permissible_locations: Array.isArray(node.permissible_locations) ? [...node.permissible_locations] : [],
        };
        return forceRoot ? { all: [single] } : single;
      }
    }

    const children = [];
    if ('event' in node) {
      const eventNode = normalizeTriggerNode({ event: node.event }, false);
      if (triggerNodeMode(eventNode) === 'all') children.push(...eventNode.all);
      else children.push(eventNode);
    }
    if ('time' in node) children.push({ time: normalizeTimeRule(node.time) });
    if ('protocol_completion' in node) {
      children.push({ protocol_completion: normalizeProtocolCompletionRule(node.protocol_completion) });
    }
    if ('permissible_locations' in node) {
      children.push({ permissible_locations: Array.isArray(node.permissible_locations) ? [...node.permissible_locations] : [] });
    }
    if ('all' in node) children.push({ all: (Array.isArray(node.all) ? node.all : []).map((child) => normalizeTriggerNode(child, false)) });
    if ('any' in node) children.push({ any: (Array.isArray(node.any) ? node.any : []).map((child) => normalizeTriggerNode(child, false)) });

    if (!children.length) return forceRoot ? {} : makeDefaultTriggerNode('event');
    return { all: children };
  }

  function parseTriggerPath(path) {
    if (!path) return [];
    return String(path).split('/').filter(Boolean).map((segment) => (/^\d+$/.test(segment) ? Number(segment) : segment));
  }

  function joinTriggerPath(path, ...parts) {
    return [path, ...parts].filter((part) => part !== '' && part != null).join('/');
  }

  function getTriggerNodeAtPath(protocol, path) {
    let node = ensureTriggers(protocol);
    for (const segment of parseTriggerPath(path)) {
      if (node == null) return null;
      node = node[segment];
    }
    return node;
  }

  function getTriggerParentAtPath(protocol, path) {
    const segments = parseTriggerPath(path);
    const key = segments.pop();
    let node = ensureTriggers(protocol);
    for (const segment of segments) {
      if (node == null) return { parent: null, key: null };
      node = node[segment];
    }
    return { parent: node, key };
  }

  function ensureTriggerRoot(protocol, defaultMode = 'all') {
    const triggers = ensureTriggers(protocol);
    if (!Object.keys(triggers).length) {
      protocol.high_level.triggers = { [defaultMode]: [] };
    }
    return protocol.high_level.triggers;
  }

  function triggerGroupItems(node) {
    const mode = triggerNodeMode(node);
    return { mode, items: mode ? node[mode] : [] };
  }

  function addTriggerNode(protocol, groupPath, kind) {
    const group = groupPath ? getTriggerNodeAtPath(protocol, groupPath) : ensureTriggerRoot(protocol, 'all');
    const { mode, items } = triggerGroupItems(group);
    if (!mode) throw new Error('Trigger target is not a group.');
    items.push(makeDefaultTriggerNode(kind));
  }

  function removeTriggerNode(protocol, path) {
    const { parent, key } = getTriggerParentAtPath(protocol, path);
    if (parent == null || key == null) return;
    if (Array.isArray(parent)) parent.splice(Number(key), 1);
    else delete parent[key];
    const triggers = ensureTriggers(protocol);
    const mode = triggerNodeMode(triggers);
    if (mode && Array.isArray(triggers[mode]) && triggers[mode].length === 0) {
      protocol.high_level.triggers = {};
    }
  }

  function setTriggerGroupMode(protocol, path, nextMode) {
    const group = path ? getTriggerNodeAtPath(protocol, path) : ensureTriggerRoot(protocol, nextMode);
    const { mode, items } = triggerGroupItems(group);
    if (mode === nextMode) return;
    delete group[mode];
    group[nextMode] = items;
  }

  function hasTriggerKind(triggerBlock, kind) {
    if (!triggerBlock || typeof triggerBlock !== 'object' || Array.isArray(triggerBlock)) return false;
    if (kind === 'time' && triggerBlock.time) return true;
    if (kind === 'event' && triggerBlock.event) return true;
    if (kind === 'protocol_completion' && triggerBlock.protocol_completion) return true;
    for (const groupKey of ['all', 'any']) {
      const children = triggerBlock[groupKey];
      if (Array.isArray(children) && children.some((child) => hasTriggerKind(child, kind))) return true;
    }
    return false;
  }

  function triggerStats(triggerBlock) {
    const stats = { events: 0, times: 0, locations: 0, completions: 0, groups: 0 };
    if (!triggerBlock || typeof triggerBlock !== 'object' || Array.isArray(triggerBlock)) return stats;
    if (triggerBlock.event) stats.events += Array.isArray(triggerBlock.event) ? triggerBlock.event.length : 1;
    if (triggerBlock.time) stats.times += 1;
    if (triggerBlock.permissible_locations) stats.locations += 1;
    if (triggerBlock.protocol_completion) stats.completions += 1;
    for (const groupKey of ['all', 'any']) {
      const children = triggerBlock[groupKey];
      if (Array.isArray(children)) {
        stats.groups += 1;
        children.forEach((child) => {
          const childStats = triggerStats(child);
          stats.events += childStats.events;
          stats.times += childStats.times;
          stats.locations += childStats.locations;
          stats.completions += childStats.completions;
          stats.groups += childStats.groups;
        });
      }
    }
    return stats;
  }

  function makeDefaultParamValue(param) {
    switch (param.kind) {
      case 'execution_location':
        return 'current';
      case 'integer':
        return param.name === 'num_attempts' ? 3 : 1;
      case 'location':
        return Object.keys(state.config?.locations || {})[0] || '';
      case 'audio_asset':
        return (state.metadata?.user_assets?.audio_files || [])[0] || '';
      case 'video_asset':
        return (state.metadata?.user_assets?.video_files || [])[0] || '';
      case 'number':
      case 'duration':
      case 'number_or_duration':
        return 0;
      default:
        if (param.name === 'position_state_key') return 'position';
        return '';
    }
  }

  function makeDefaultTreeStep(treeName = 'play_text') {
    const meta = metadataForTree(treeName);
    const tree_params = {};
    for (const p of meta.params || []) {
      if (p.required) tree_params[p.name] = makeDefaultParamValue(p);
    }
    if (treeName === 'play_text' && tree_params.text === '') tree_params.text = 'New message';
    if (treeName === 'move_away') {
      if (tree_params.position_state_key === '') tree_params.position_state_key = 'position';
      if (tree_params.state_key === '') tree_params.state_key = 'move_away';
    }
    if ((meta.params || []).some((p) => p.name === 'execution_location')) {
      tree_params.execution_location = tree_params.execution_location || 'current';
    }
    return { tree_name: treeName, tree_params };
  }

  function makeDefaultConfirmationStep() {
    return {
      confirmation: {
        question: 'Do you want me to continue?',
        execution_location: 'current',
        on_yes: [makeDefaultTreeStep('play_text')],
        on_no: [makeDefaultTreeStep('play_text')],
      },
    };
  }

  function getProtocols() {
    return state.config?.protocols || {};
  }

  function getGenericProtocolNamesRaw() {
    return Object.entries(getProtocols())
      .filter(([, p]) => p && p.runner === 'GenericProtocol')
      .map(([name]) => name);
  }

  function getGenericProtocolNames() {
    return getGenericProtocolNamesRaw().sort();
  }

  function getFilteredGenericProtocols() {
    let items = getGenericProtocolNamesRaw().map((name) => ({ name, protocol: getProtocols()[name] }));
    const search = state.ui.listSearch.trim().toLowerCase();
    if (search) {
      items = items.filter(({ name, protocol }) => {
        const hay = `${name} ${triggerSummary(protocol)}`.toLowerCase();
        return hay.includes(search);
      });
    }
    if (state.ui.filters.hasTime) {
      items = items.filter(({ protocol }) => hasTriggerKind(protocol?.high_level?.triggers || {}, 'time'));
    }
    if (state.ui.filters.hasEvent) {
      items = items.filter(({ protocol }) => hasTriggerKind(protocol?.high_level?.triggers || {}, 'event'));
    }
    const sort = state.ui.listSort;
    items.sort((a, b) => {
      if (sort === 'priority_asc') {
        return (a.protocol?.high_level?.priority ?? 9999) - (b.protocol?.high_level?.priority ?? 9999)
          || a.name.localeCompare(b.name);
      }
      if (sort === 'recent') {
        const ra = state.ui.recentProtocols.indexOf(a.name);
        const rb = state.ui.recentProtocols.indexOf(b.name);
        const va = ra === -1 ? Number.MAX_SAFE_INTEGER : ra;
        const vb = rb === -1 ? Number.MAX_SAFE_INTEGER : rb;
        return va - vb || a.name.localeCompare(b.name);
      }
      return a.name.localeCompare(b.name);
    });
    return items;
  }

  function getSpecialProtocolNames() {
    return Object.entries(getProtocols())
      .filter(([, p]) => p && p.runner !== 'GenericProtocol')
      .map(([name, p]) => ({ name, runner: p.runner || 'Unknown' }))
      .sort((a, b) => a.name.localeCompare(b.name));
  }

  function selectedProtocolObj() {
    return state.selectedProtocol ? getProtocols()[state.selectedProtocol] : null;
  }

  function ensureGenericShapes(protocol) {
    protocol.high_level ||= {};
    protocol.high_level.priority ??= 10;
    protocol.high_level.triggers ||= {};
    protocol.high_level.reset_pattern ||= { type: 'eod' };
    protocol.action ||= {};
    protocol.action.steps ||= [];
  }

  function currentSteps(protocol = selectedProtocolObj()) {
    if (!protocol) return [];
    ensureGenericShapes(protocol);
    return protocol.action.steps;
  }

  function getBranchSteps(step, branchName) {
    step.confirmation ||= { question: '', on_yes: [], on_no: [] };
    step.confirmation[branchName] ||= [];
    return step.confirmation[branchName];
  }

  function markRecentProtocol(name) {
    if (!name) return;
    state.ui.recentProtocols = [name, ...state.ui.recentProtocols.filter((n) => n !== name)].slice(0, 30);
  }

  function setDirty(flag = true) {
    state.dirty = flag;
    if (flag) {
      if (state.validation.status === 'ok') {
        state.validation.status = 'idle';
      }
    }
    updateStatusChip();
  }

  function validationStatusLabel() {
    if (state.loading) return 'Loading…';
    if (state.validation.status === 'error') return 'Validation error';
    if (state.validation.status === 'ok') return 'Validated';
    if (state.lastSavedAt && !state.dirty) return 'Saved';
    if (state.dirty) return 'Unsaved';
    return 'Ready';
  }

  function updateStatusChip(text) {
    const label = text || validationStatusLabel();
    els.statusChip.textContent = label;
    els.statusChip.className = 'chip';
    if (state.validation.status === 'error') els.statusChip.classList.add('chip-error');
    else if (state.dirty) els.statusChip.classList.add('chip-warn');
    else if (state.lastSavedAt || state.validation.status === 'ok') els.statusChip.classList.add('chip-ok');
  }

  function renderNav() {
    for (const btn of els.navItems) {
      btn.classList.toggle('active', btn.dataset.route === state.route);
    }
  }

  function renderSidebarMode() {
    els.appShell.classList.toggle('sidebar-collapsed', !!state.ui.sidebarCollapsed);
    if (els.sidebarToggle) {
      els.sidebarToggle.setAttribute(
        'aria-label',
        state.ui.sidebarCollapsed ? 'Expand navigation' : 'Collapse navigation',
      );
      els.sidebarToggle.setAttribute(
        'title',
        state.ui.sidebarCollapsed ? 'Expand navigation' : 'Collapse navigation',
      );
    }
  }

  function routeMeta(route) {
    if (route === 'protocol-designer') {
      return {
        title: 'Protocol Designer',
        subtitle: 'Edit GenericProtocol entries from the active house YAML',
      };
    }
    if (route === 'protocol-history') {
      return {
        title: 'Protocol History',
        subtitle: 'View today\u2019s protocol runs and live state',
      };
    }
    return { title: 'Home', subtitle: 'Live robot status overview' };
  }

  function setRoute(route) {
    // Stop home polling when leaving home page
    if (state.route === 'home' && route !== 'home') {
      stopHomePoll();
    }
    // Disconnect SSE when leaving history page
    if (state.route === 'protocol-history' && route !== 'protocol-history') {
      disconnectSSE();
    }
    state.route = route;
    const meta = routeMeta(route);
    els.pageTitle.textContent = meta.title;
    els.pageSubtitle.textContent = meta.subtitle;
    renderNav();
    renderRoute();
    // Start home polling when entering home page
    if (route === 'home') {
      startHomePoll();
    }
  }

  function captureDesignerScroll() {
    const listPanel = els.routeView.querySelector('.designer-list-panel');
    const editorPanel = els.routeView.querySelector('.designer-editor-panel');
    if (listPanel) state.ui.scroll.list = listPanel.scrollTop;
    if (editorPanel) state.ui.scroll.editor = editorPanel.scrollTop;
  }

  function restoreDesignerScroll() {
    const listPanel = els.routeView.querySelector('.designer-list-panel');
    const editorPanel = els.routeView.querySelector('.designer-editor-panel');
    if (listPanel) listPanel.scrollTop = state.ui.scroll.list || 0;
    if (editorPanel) editorPanel.scrollTop = state.ui.scroll.editor || 0;
  }

  function renderRoute() {
    const shouldPreserveDesignerScroll = state.route === 'protocol-designer';
    if (shouldPreserveDesignerScroll) captureDesignerScroll();
    if (state.loading) {
      els.routeView.innerHTML = '<div class="blank-card">Loading dashboard…</div>';
      return;
    }
    if (state.bootError) {
      els.routeView.innerHTML = `<div class="panel"><h2>Bootstrap Error</h2><p class="muted">${escapeHtml(state.bootError)}</p></div>`;
      return;
    }
    if (state.route === 'protocol-designer') {
      renderProtocolDesigner();
      restoreDesignerScroll();
      return;
    }
    if (state.route === 'protocol-history') {
      renderProtocolHistory();
      return;
    }
    renderHomeDashboard();
  }

  function triggerSummary(protocol) {
    const triggers = protocol?.high_level?.triggers || {};
    const stats = triggerStats(triggers);
    const bits = [];
    if (stats.events) bits.push(`event:${stats.events}`);
    if (stats.times) bits.push(`time:${stats.times}`);
    if (stats.completions) bits.push(`completion:${stats.completions}`);
    if (stats.locations) bits.push(`loc:${stats.locations}`);
    if (stats.groups) bits.push(`groups:${stats.groups}`);
    return bits.join(' · ') || 'No triggers';
  }

  function renderProtocolListPanel() {
    const items = getFilteredGenericProtocols();
    const specials = getSpecialProtocolNames();

    const listHtml = items.map(({ name, protocol }) => {
      const active = name === state.selectedProtocol ? ' active' : '';
      const prio = protocol?.high_level?.priority ?? '-';
      return `
        <button class="protocol-item${active}" data-action="select-protocol" data-name="${escapeAttr(name)}">
          <div class="title">${escapeHtml(name)}</div>
          <div class="meta">Priority ${prio} · ${escapeHtml(triggerSummary(protocol))}</div>
        </button>`;
    }).join('');

    const specialList = specials.length
      ? specials.map((p) => `<div class="inline-note">${escapeHtml(p.name)} <span class="param-chip">${escapeHtml(p.runner)}</span></div>`).join('')
      : '<div class="inline-note">No special protocol runners</div>';

    return `
      <div class="panel stack designer-list-panel">
        <div class="section-header">
          <h2 style="margin:0; font-size:18px;">Protocols</h2>
          <button class="btn small primary" data-action="add-protocol">Add</button>
        </div>

        <div class="section stack">
          <div class="row">
            <div class="col-12">
              <label>Search</label>
              <input type="text" data-action="list-search" value="${escapeAttr(state.ui.listSearch)}" placeholder="Search by name or trigger summary" />
            </div>
            <div class="col-6">
              <label>Sort</label>
              <select data-action="list-sort">
                <option value="name_asc" ${state.ui.listSort === 'name_asc' ? 'selected' : ''}>Name (A-Z)</option>
                <option value="priority_asc" ${state.ui.listSort === 'priority_asc' ? 'selected' : ''}>Priority</option>
                <option value="recent" ${state.ui.listSort === 'recent' ? 'selected' : ''}>Recently edited</option>
              </select>
            </div>
            <div class="col-6">
              <label>Filters</label>
              <div class="filter-grid">
                <label class="checkbox-line"><input type="checkbox" data-action="list-filter" data-key="hasTime" ${state.ui.filters.hasTime ? 'checked' : ''}/> <span>Time</span></label>
                <label class="checkbox-line"><input type="checkbox" data-action="list-filter" data-key="hasEvent" ${state.ui.filters.hasEvent ? 'checked' : ''}/> <span>Event</span></label>
              </div>
            </div>
          </div>
        </div>

        <div class="protocol-list">${listHtml || '<div class="inline-note">No GenericProtocol entries match the current filters.</div>'}</div>

        <div class="section stack">
          <button class="btn small ghost" data-action="toggle-special-panel">${state.ui.specialCollapsed ? 'Show' : 'Hide'} Special Runners</button>
          ${state.ui.specialCollapsed ? '' : `<div class="stack">${specialList}</div>`}
        </div>
      </div>`;
  }

  function triggerRuleLabel(node) {
    if (node?.event) return 'Event';
    if (node?.time) return 'Time';
    if (node?.protocol_completion) return 'Protocol';
    if (node?.permissible_locations) return 'Location Filter';
    const mode = triggerNodeMode(node);
    if (mode === 'all') return 'AND Group';
    if (mode === 'any') return 'OR Group';
    return 'Rule';
  }

  function clientValidationForSelected() {
    const protocol = selectedProtocolObj();
    if (!protocol) return { fieldErrors: {}, messages: [] };
    ensureGenericShapes(protocol);
    const errors = {};
    const messages = [];
    const push = (key, message) => {
      errors[key] = message;
      messages.push(message);
    };

    const triggers = ensureTriggers(protocol);
    const hhmm = /^([01]\d|2[0-3]):([0-5]\d)$/;
    const validateTriggerNode = (node, pathLabel = 'triggers') => {
      if (!node || typeof node !== 'object' || Array.isArray(node)) {
        push(pathLabel, 'Trigger rule is invalid.');
        return;
      }
      const mode = triggerNodeMode(node);
      if (mode) {
        const items = node[mode] || [];
        if (!items.length) push(pathLabel, 'Trigger group must contain at least one clause.');
        items.forEach((child, idx) => validateTriggerNode(child, `${pathLabel}.${mode}[${idx}]`));
        return;
      }
      if (node.event) {
        const evt = node.event;
        if (!evt?.state || !String(evt.state).trim()) {
          push(`${pathLabel}.event.state`, 'Event trigger key is required.');
          return;
        }
        if (evt.state === 'robot_location_xy') {
          if (!(typeof evt.within_m === 'number') || Number.isNaN(evt.within_m) || evt.within_m <= 0) {
            push(`${pathLabel}.event.within_m`, 'Proximity trigger radius must be > 0.');
          }
          const p = evt.point_xy;
          if (!Array.isArray(p) || p.length !== 2 || p.some((v) => typeof v !== 'number' || Number.isNaN(v))) {
            push(`${pathLabel}.event.point_xy`, 'Proximity trigger target XY must be two numbers.');
          }
        }
        if (evt.edge) {
          if (!['rising', 'falling'].includes(evt.edge)) {
            push(`${pathLabel}.event.edge`, 'Edge must be rising or falling.');
          }
          if (typeof evt.value !== 'boolean') {
            push(`${pathLabel}.event.value`, 'Edge-based event value must be boolean.');
          } else if (evt.edge === 'rising' && evt.value !== true) {
            push(`${pathLabel}.event.value`, 'Rising edge requires expected value true.');
          } else if (evt.edge === 'falling' && evt.value !== false) {
            push(`${pathLabel}.event.value`, 'Falling edge requires expected value false.');
          }
        }
        return;
      }
      if (node.time) {
        const t = node.time || {};
        if (t.from && !hhmm.test(String(t.from))) push(`${pathLabel}.time.from`, 'Time Trigger "From" must be HH:MM');
        if (t.to && !hhmm.test(String(t.to))) push(`${pathLabel}.time.to`, 'Time Trigger "To" must be HH:MM');
        if (t.from && t.to && hhmm.test(String(t.from)) && hhmm.test(String(t.to)) && t.from > t.to) {
          push(`${pathLabel}.time`, 'Time window From must be earlier than or equal to To.');
        }
        return;
      }
      if (node.protocol_completion) {
        const rule = node.protocol_completion || {};
        if (!rule.protocol || !String(rule.protocol).trim()) {
          push(`${pathLabel}.protocol_completion.protocol`, 'Source protocol is required.');
        }
        if (!Array.isArray(rule.statuses) || !rule.statuses.length) {
          push(`${pathLabel}.protocol_completion.statuses`, 'Select at least one terminal status.');
        }
        const within = Number(rule.within_seconds);
        if (!Number.isFinite(within) || within <= 0) {
          push(`${pathLabel}.protocol_completion.within_seconds`, 'Within seconds must be greater than 0.');
        }
        if (rule.after_seconds !== undefined && rule.after_seconds !== '') {
          const after = Number(rule.after_seconds);
          if (!Number.isFinite(after) || after < 0) {
            push(`${pathLabel}.protocol_completion.after_seconds`, 'After seconds must be 0 or greater.');
          } else if (Number.isFinite(within) && within > 0 && after > within) {
            push(`${pathLabel}.protocol_completion.after_seconds`, 'After seconds must be less than or equal to Within seconds.');
          }
        }
        return;
      }
      if (node.permissible_locations) return;
      push(pathLabel, 'Trigger clause is empty.');
    };
    if (Object.keys(triggers).length) validateTriggerNode(triggers);

    const reset = protocol.high_level?.reset_pattern || {};
    if (reset.type === 'periodic') {
      const h = Number(reset.hours || 0);
      const m = Number(reset.minutes || 0);
      if ((Number.isNaN(h) || h < 0) || (Number.isNaN(m) || m < 0) || (h === 0 && m === 0)) {
        push('reset.periodic', 'Periodic reset requires hours > 0 or minutes > 0');
      }
    }

    return { fieldErrors: errors, messages };
  }

  function fieldError(key) {
    return clientValidationForSelected().fieldErrors[key] || '';
  }

  function errorHtml(key) {
    const msg = fieldError(key);
    return `<div class="field-error">${msg ? escapeHtml(msg) : ''}</div>`;
  }

  function pathLabelFromPath(path) {
    const segments = parseTriggerPath(path);
    let out = '';
    segments.forEach((segment) => {
      if (typeof segment === 'number') out += `[${segment}]`;
      else out += out ? `.${segment}` : segment;
    });
    return out;
  }

  function renderTriggerEventStateSelect(evt, path) {
    const description = robotStateSpec(evt?.state)?.description || '';
    const options = [
      `<option value="" ${!(evt?.state) ? 'selected' : ''}>Select Event</option>`,
      ...robotStateSpecs().map((s) => `<option value="${escapeAttr(s.name)}" ${s.name === (evt?.state || '') ? 'selected' : ''}>${escapeHtml(s.name)}</option>`),
    ].join('');
    return `<select data-action="trigger-event-state" data-path="${escapeAttr(path)}" title="${escapeAttr(description)}">${options}</select>`;
  }

  function boolTriggerSelectionValue(evt) {
    if (evt?.edge === 'rising') return 'rising';
    if (evt?.edge === 'falling') return 'falling';
    return evt?.value === true ? 'true' : 'false';
  }

  function renderTriggerScalarEventNode(evt, path, kind, isLocationValue) {
    const rawValue = evt?.value;
    const op = kind === 'number' ? (evt?.op || '=') : '=';
    let textValue = '';
    if (kind === 'bool') textValue = rawValue ? 'true' : 'false';
    else textValue = rawValue == null ? '' : String(rawValue);
    const locationOptions = isLocationValue ? locationValueOptionsForState(evt?.state) : [];

    return `
      <div class="section event-row trigger-clause-card">
        <div class="trigger-clause-head">
          <div>
            <div class="eyebrow">Event Rule</div>
            <h3>Match a robot state value</h3>
          </div>
          <button class="btn small danger" data-action="trigger-remove-node" data-path="${escapeAttr(path)}">Remove</button>
        </div>
        <div class="row">
          <div class="col-4">
            <label>Event Key</label>
            ${renderTriggerEventStateSelect(evt, path)}
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.event.state`)}
          </div>
          <div class="col-2">
            <label>Op</label>
            ${kind === 'number' ? `
              <select data-action="trigger-event-op" data-path="${escapeAttr(path)}">
                <option value="=" ${op === '=' ? 'selected' : ''}>=</option>
                <option value="!=" ${op === '!=' ? 'selected' : ''}>!=</option>
                <option value=">" ${op === '>' ? 'selected' : ''}>&gt;</option>
                <option value=">=" ${op === '>=' ? 'selected' : ''}>&gt;=</option>
                <option value="<" ${op === '<' ? 'selected' : ''}>&lt;</option>
                <option value="<=" ${op === '<=' ? 'selected' : ''}>&lt;=</option>
              </select>` : '<input type="text" value="=" disabled />'}
          </div>
          <div class="col-4">
            <label>Expected Value</label>
            ${kind === 'bool' ? `
              <select data-action="trigger-event-value" data-path="${escapeAttr(path)}" data-type="bool">
                <option value="true" ${boolTriggerSelectionValue(evt) === 'true' ? 'selected' : ''}>true</option>
                <option value="false" ${boolTriggerSelectionValue(evt) === 'false' ? 'selected' : ''}>false</option>
                <option value="rising" ${boolTriggerSelectionValue(evt) === 'rising' ? 'selected' : ''}>rising (false to true)</option>
                <option value="falling" ${boolTriggerSelectionValue(evt) === 'falling' ? 'selected' : ''}>falling (true to false)</option>
              </select>` : isLocationValue ? `
              <select data-action="trigger-event-value" data-path="${escapeAttr(path)}" data-type="string">
                ${locationOptions.map((loc) => `<option value="${escapeAttr(loc)}" ${String(rawValue ?? '') === loc ? 'selected' : ''}>${escapeHtml(loc)}</option>`).join('')}
              </select>` : `
              <input type="${kind === 'number' ? 'number' : 'text'}" data-action="trigger-event-value" data-path="${escapeAttr(path)}" data-type="${kind}" value="${escapeAttr(textValue)}" />`
            }
          </div>
        </div>
      </div>`;
  }

  function renderTriggerXYProximityNode(evt, path) {
    const p = Array.isArray(evt?.point_xy) && evt.point_xy.length === 2 ? evt.point_xy : [];
    const pointCsv = p.length === 2 ? `${p[0]}, ${p[1]}` : '';
    return `
      <div class="section event-row event-row-special trigger-clause-card">
        <div class="trigger-clause-head">
          <div>
            <div class="eyebrow">Event Rule</div>
            <h3>Proximity Trigger</h3>
          </div>
          <button class="btn small danger" data-action="trigger-remove-node" data-path="${escapeAttr(path)}">Remove</button>
        </div>
        <div class="row">
          <div class="col-4">
            <label>Event Key</label>
            ${renderTriggerEventStateSelect(evt, path)}
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.event.state`)}
          </div>
          <div class="col-3">
            <label>Within (m)</label>
            <input type="number" step="0.01" min="0" data-action="trigger-event-within-m" data-path="${escapeAttr(path)}" value="${escapeAttr(evt?.within_m ?? 1.0)}" />
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.event.within_m`)}
          </div>
          <div class="col-5">
            <label>Target XY (x, y)</label>
            <input type="text" data-action="trigger-event-point-xy-csv" data-path="${escapeAttr(path)}" value="${escapeAttr(pointCsv)}" placeholder="2.10, 3.45" />
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.event.point_xy`)}
          </div>
          <div class="col-12">
            <div class="inline-note">Enter two floats separated by comma. Trigger fires when robot is within the radius of this point.</div>
          </div>
        </div>
      </div>`;
  }

  function renderTriggerEventNode(node, path) {
    const evt = node.event || {};
    const kind = eventKindForState(evt?.state, evt?.value);
    const uiKind = eventUiKindForState(evt?.state);
    if (uiKind === 'xy_proximity') return renderTriggerXYProximityNode(evt, path);
    if (uiKind === 'location_name') return renderTriggerScalarEventNode(evt, path, 'string', true);
    return renderTriggerScalarEventNode(evt, path, kind, false);
  }

  function renderTriggerTimeNode(node, path) {
    const time = normalizeTimeRule(node.time || {});
    const days = new Set(time.day || []);
    const dayChecks = (state.metadata?.day_options || []).map((day) => `
      <label class="checkbox-line">
        <input type="checkbox" data-action="trigger-time-day-toggle" data-path="${escapeAttr(path)}" data-day="${day}" ${days.has(day) ? 'checked' : ''}/>
        <span>${capitalize(day)}</span>
      </label>`).join('');

    return `
      <div class="section trigger-clause-card">
        <div class="trigger-clause-head">
          <div>
            <div class="eyebrow">Time Rule</div>
            <h3>Time Window Match</h3>
          </div>
          <button class="btn small danger" data-action="trigger-remove-node" data-path="${escapeAttr(path)}">Remove</button>
        </div>
        <div class="row">
          <div class="col-6">
            <label>From</label>
            ${renderTimeSelect('from', time.from || '', path)}
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.time.from`)}
          </div>
          <div class="col-6">
            <label>To</label>
            ${renderTimeSelect('to', time.to || '', path)}
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.time.to`)}
          </div>
          <div class="col-12">
            <label>Days (empty = every day)</label>
            <div class="day-grid">${dayChecks}</div>
          </div>
        </div>
      </div>`;
  }

  function renderTriggerLocationNode(node, path) {
    const locations = Object.keys(state.config?.locations || {});
    const selected = new Set(node.permissible_locations || []);
    return `
      <div class="section trigger-clause-card">
        <div class="trigger-clause-head">
          <div>
            <div class="eyebrow">Location Rule</div>
            <h3>Permissible Locations</h3>
          </div>
          <button class="btn small danger" data-action="trigger-remove-node" data-path="${escapeAttr(path)}">Remove</button>
        </div>
        <div class="day-grid">
          ${locations.map((loc) => `
            <label class="checkbox-line">
              <input type="checkbox" data-action="trigger-location-toggle" data-path="${escapeAttr(path)}" data-location="${escapeAttr(loc)}" ${selected.has(loc) ? 'checked' : ''}/>
              <span>${escapeHtml(loc)}</span>
            </label>`).join('')}
        </div>
      </div>`;
  }

  function renderTriggerProtocolCompletionNode(node, path) {
    const rule = normalizeProtocolCompletionRule(node.protocol_completion || {});
    const protocolOptions = protocolTriggerOptions(rule.protocol);
    const statuses = [
      { value: 'completed', label: 'Success' },
      { value: 'failed', label: 'Failure' },
    ];
    const selectedStatuses = new Set(rule.statuses || []);
    return `
      <div class="section trigger-clause-card">
        <div class="trigger-clause-head">
          <div>
            <div class="eyebrow">Protocol Rule</div>
            <h3>Follow Another Protocol</h3>
          </div>
          <button class="btn small danger" data-action="trigger-remove-node" data-path="${escapeAttr(path)}">Remove</button>
        </div>
        <div class="row">
          <div class="col-5">
            <label>Protocol</label>
            <select data-action="trigger-protocol-completion-protocol" data-path="${escapeAttr(path)}">
              <option value="" ${!rule.protocol ? 'selected' : ''}>Select protocol</option>
              ${protocolOptions.map((opt) => `<option value="${escapeAttr(opt)}" ${rule.protocol === opt ? 'selected' : ''}>${escapeHtml(opt)}</option>`).join('')}
            </select>
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.protocol_completion.protocol`)}
          </div>
          <div class="col-2">
            <label title="Start matching this rule only after this many seconds from the selected protocol completion.">After (sec)</label>
            <input type="number" min="0" step="1" data-action="trigger-protocol-completion-after" data-path="${escapeAttr(path)}" value="${escapeAttr(rule.after_seconds ?? '')}" placeholder="0" title="Start matching this rule only after this many seconds from the selected protocol completion." />
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.protocol_completion.after_seconds`)}
          </div>
          <div class="col-2">
            <label title="Stop matching this rule after this many seconds from the selected protocol completion.">Within (sec)</label>
            <input type="number" min="1" step="1" data-action="trigger-protocol-completion-within" data-path="${escapeAttr(path)}" value="${escapeAttr(rule.within_seconds ?? 120)}" title="Stop matching this rule after this many seconds from the selected protocol completion." />
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.protocol_completion.within_seconds`)}
          </div>
          <div class="col-3">
            <label>Trigger On</label>
            <div class="status-checkboxes">
              ${statuses.map((status) => `
                <label class="checkbox-line compact">
                  <input
                    type="checkbox"
                    data-action="trigger-protocol-completion-status"
                    data-path="${escapeAttr(path)}"
                    data-status="${escapeAttr(status.value)}"
                    ${selectedStatuses.has(status.value) ? 'checked' : ''}
                  />
                  <span>${escapeHtml(status.label)}</span>
                </label>`).join('')}
            </div>
            ${errorHtml(`triggers.${pathLabelFromPath(path)}.protocol_completion.statuses`)}
          </div>
        </div>
      </div>`;
  }

  function renderTriggerAddBar(path, compact = false) {
    return `
      <div class="trigger-add-bar ${compact ? 'compact' : ''}">
        <button class="btn small primary" data-action="trigger-add-node" data-path="${escapeAttr(path)}" data-kind="event">Add Event</button>
        <button class="btn small ghost" data-action="trigger-add-node" data-path="${escapeAttr(path)}" data-kind="time">Add Time</button>
        <button class="btn small ghost" data-action="trigger-add-node" data-path="${escapeAttr(path)}" data-kind="protocol_completion">Add Protocol</button>
        <button class="btn small ghost" data-action="trigger-add-node" data-path="${escapeAttr(path)}" data-kind="permissible_locations">Add Location</button>
        <button class="btn small ghost" data-action="trigger-add-node" data-path="${escapeAttr(path)}" data-kind="all">Add AND Group</button>
        <button class="btn small ghost" data-action="trigger-add-node" data-path="${escapeAttr(path)}" data-kind="any">Add OR Group</button>
      </div>`;
  }

  function renderTriggerGroupNode(node, path, depth = 0, isRoot = false) {
    const { mode, items } = triggerGroupItems(node);
    const title = isRoot ? 'Protocol Trigger Logic' : (mode === 'all' ? 'AND Group' : 'OR Group');
    const desc = mode === 'all'
      ? 'All clauses in this group must match.'
      : 'Any one clause in this group may match.';
    return `
      <div class="section trigger-group-card ${isRoot ? 'trigger-group-root' : ''}" data-depth="${depth}">
        <div class="trigger-group-head">
          <div>
            <div class="eyebrow">${isRoot ? 'Root Rule' : 'Nested Group'}</div>
            <h3>${escapeHtml(title)}</h3>
            <div class="inline-note">${escapeHtml(desc)}</div>
          </div>
          <div class="trigger-inline-actions">
            <select data-action="trigger-group-mode" data-path="${escapeAttr(path)}">
              <option value="all" ${mode === 'all' ? 'selected' : ''}>AND</option>
              <option value="any" ${mode === 'any' ? 'selected' : ''}>OR</option>
            </select>
            ${isRoot ? '' : `<button class="btn small danger" data-action="trigger-remove-node" data-path="${escapeAttr(path)}">Remove Group</button>`}
          </div>
        </div>
        <div class="trigger-group-body">
          ${items.length
            ? items.map((child, idx) => renderTriggerNode(child, joinTriggerPath(path, mode, idx), depth + 1)).join('')
            : '<div class="inline-note">No clauses in this group yet.</div>'}
        </div>
        ${renderTriggerAddBar(path, items.length > 0)}
      </div>`;
  }

  function renderTriggerNode(node, path, depth = 0) {
    const mode = triggerNodeMode(node);
    if (mode) return renderTriggerGroupNode(node, path, depth, false);
    if (node?.event) return renderTriggerEventNode(node, path);
    if (node?.time) return renderTriggerTimeNode(node, path);
    if (node?.protocol_completion) return renderTriggerProtocolCompletionNode(node, path);
    if (node?.permissible_locations) return renderTriggerLocationNode(node, path);
    return `<div class="section trigger-clause-card"><div class="inline-note">Unknown trigger rule: ${escapeHtml(triggerRuleLabel(node))}</div></div>`;
  }

  function renderTriggerBuilder(protocol) {
    const triggers = ensureTriggers(protocol);
    const stats = triggerStats(triggers);
    const hasRoot = !!triggerNodeMode(triggers);
    const bits = [];
    bits.push(`${stats.events} event${stats.events === 1 ? '' : 's'}`);
    bits.push(`${stats.times} time rule${stats.times === 1 ? '' : 's'}`);
    bits.push(`${stats.completions} completion rule${stats.completions === 1 ? '' : 's'}`);
    bits.push(`${stats.locations} location filter${stats.locations === 1 ? '' : 's'}`);
    return `
      <div class="section stack trigger-builder-shell">
        <div class="inline-note trigger-overview">${escapeHtml(bits.join(' · '))}</div>
        ${hasRoot
          ? renderTriggerGroupNode(triggers, '', 0, true)
          : `
            <div class="section trigger-empty-state">
              <div>
                <h3>Build Trigger Logic</h3>
                <div class="inline-note">Start with a rule clause, then combine clauses with AND / OR groups as needed.</div>
              </div>
              ${renderTriggerAddBar('', false)}
            </div>`}
      </div>`;
  }

  function renderResetSection(protocol) {
    const reset = protocol.high_level?.reset_pattern || { type: 'eod' };
    const periodic = reset.type === 'periodic';
    return `
      <div class="section stack">
        <div class="inline-note">Scheduling priority and protocol re-trigger reset behavior.</div>
        <div class="row">
          <div class="col-4">
            <label>Priority</label>
            <input type="number" data-action="priority" value="${escapeAttr(protocol.high_level?.priority ?? 10)}" />
          </div>
          <div class="${periodic ? 'col-4' : 'col-8'}">
            <label>Reset Pattern</label>
            <select data-action="reset-type">
              ${(state.metadata?.reset_types || ['eod', 'instant', 'periodic']).map((t) => `<option value="${t}" ${reset.type === t ? 'selected' : ''}>${t}</option>`).join('')}
            </select>
          </div>
          ${periodic ? `
          <div class="col-2">
            <label>Hours</label>
            <input type="number" data-action="reset-periodic-hours" value="${escapeAttr(reset.hours ?? '')}" />
          </div>
          <div class="col-2">
            <label>Minutes</label>
            <input type="number" data-action="reset-periodic-minutes" value="${escapeAttr(reset.minutes ?? '')}" />
          </div>` : ''}
          <div class="col-12">
            ${errorHtml('reset.periodic')}
          </div>
        </div>
      </div>`;
  }

  function renderTreeParams(step, stepCtx) {
    const treeName = step.tree_name || '';
    const meta = metadataForTree(treeName);
    const params = meta.params || [];
    if (!params.length) return '<div class="inline-note">No parameters for this action.</div>';
    const locationNames = Object.keys(state.config?.locations || {});
    return params.map((param) => {
      const value = step.tree_params?.[param.name];
      const requiredChip = param.required ? '<span class="param-chip">required</span>' : '<span class="param-chip">optional</span>';
      const common = `data-action="step-tree-param" data-scope="${stepCtx.scope}" data-step-idx="${stepCtx.stepIdx}" data-param="${param.name}" ${stepCtx.parentIdx != null ? `data-parent-idx="${stepCtx.parentIdx}"` : ''}`;
      let control;
      if (param.kind === 'location') {
        control = `<select ${common}>${locationNames.map((loc) => `<option value="${escapeAttr(loc)}" ${String(value ?? '') === loc ? 'selected' : ''}>${escapeHtml(loc)}</option>`).join('')}</select>`;
      } else if (param.kind === 'audio_asset' || param.kind === 'video_asset') {
        const options = assetOptions(param.kind, value ?? '');
        const selected = String(value ?? '');
        control = `<select ${common}>${options.map((opt) => {
          const label = opt || (param.kind === 'audio_asset' ? 'Select audio' : 'Select video');
          return `<option value="${escapeAttr(opt)}" ${selected === opt ? 'selected' : ''}>${escapeHtml(label)}</option>`;
        }).join('')}</select>`;
      } else if (param.kind === 'execution_location') {
        const options = executionLocationOptions();
        const selected = String(value ?? 'current');
        control = `<select ${common}>${options.map((opt) => `<option value="${escapeAttr(opt)}" ${selected === opt ? 'selected' : ''}>${escapeHtml(opt)}</option>`).join('')}</select>`;
      } else if (param.kind === 'integer') {
        control = `<input type="number" ${common} value="${escapeAttr(value ?? '')}" />`;
      } else if (param.kind === 'number') {
        control = `<input type="number" step="any" ${common} value="${escapeAttr(value ?? '')}" />`;
      } else {
        control = `<input type="text" ${common} value="${escapeAttr(value ?? '')}" />`;
      }
      return `
        <div class="col-6">
          <label title="${escapeAttr(param.type_hint || '')}">${escapeHtml(param.name)} ${requiredChip}</label>
          ${control}
          <div class="inline-note">${escapeHtml(param.type_hint || '')}</div>
        </div>`;
    }).join('');
  }

  function stepKey(scope, idx, parentIdx = null) {
    return parentIdx == null ? `${scope}:${idx}` : `${scope}:${parentIdx}:${idx}`;
  }

  function isStepCollapsed(scope, idx, parentIdx = null) {
    return !!state.ui.stepCollapsed[stepKey(scope, idx, parentIdx)];
  }

  function setStepCollapsed(scope, idx, collapsed, parentIdx = null) {
    const key = stepKey(scope, idx, parentIdx);
    if (collapsed) state.ui.stepCollapsed[key] = true;
    else delete state.ui.stepCollapsed[key];
  }

  function stepSummary(step) {
    if (!step) return '';
    if (step.confirmation) {
      const q = step.confirmation.question || 'Confirmation';
      return `confirmation -> ${truncate(q, 52)}`;
    }
    const treeName = step.tree_name || 'tree';
    const params = step.tree_params || {};
    let detail = '';
    if (treeName === 'play_text') detail = params.text || '';
    else if (treeName === 'play_audio') detail = params.audio_path || '';
    else if (treeName === 'play_video') detail = params.video_path || '';
    else detail = Object.keys(params).length ? JSON.stringify(params) : '';
    return `${treeName}${detail ? ` -> ${truncate(String(detail), 56)}` : ''}`;
  }

  function truncate(text, n) {
    if (!text) return '';
    return text.length > n ? `${text.slice(0, n - 1)}…` : text;
  }

  function renderTreeStep(step, idx, scope = 'top', parentIdx = null) {
    const treeName = step.tree_name || '';
    const meta = metadataForTree(treeName);
    const treeNames = Object.keys(state.metadata?.run_trees || {}).sort();
    const scopeAttrs = `data-scope="${scope}" data-step-idx="${idx}" ${parentIdx != null ? `data-parent-idx="${parentIdx}"` : ''}`;
    const collapsed = isStepCollapsed(scope, idx, parentIdx);
    const titleLabel = scope === 'top' ? `Step ${idx + 1}` : `Branch Step ${idx + 1}`;

    return `
      <div class="step-card ${collapsed ? 'collapsed' : ''}">
        <div class="step-header">
          <div>
            <div class="step-title">${titleLabel} · Action</div>
            <div class="step-summary">${escapeHtml(stepSummary(step))}</div>
          </div>
          <div class="btns">
            <button class="btn small ghost btn-toggle-icon" data-action="step-toggle-collapse" ${scopeAttrs} title="${collapsed ? 'Expand' : 'Collapse'}">
              <span class="chevron-icon ${collapsed ? 'is-collapsed' : 'is-open'}" aria-hidden="true"></span>
            </button>
            <button class="btn small ghost" data-action="step-add-below" ${scopeAttrs}>+ Below</button>
            <button class="btn small ghost" data-action="step-duplicate" ${scopeAttrs}>Duplicate</button>
            <button class="btn small ghost" data-action="step-move-up" ${scopeAttrs}>↑</button>
            <button class="btn small ghost" data-action="step-move-down" ${scopeAttrs}>↓</button>
            <button class="btn small danger" data-action="step-delete" ${scopeAttrs}>Delete</button>
          </div>
        </div>
        ${collapsed ? '' : `
        <div class="row">
          <div class="col-6">
            <label>Action Name</label>
            <select data-action="step-tree-name" ${scopeAttrs} title="${escapeAttr(meta.description || '')}">
              ${treeNames.map((name) => `<option value="${escapeAttr(name)}" ${name === treeName ? 'selected' : ''}>${escapeHtml(name)}</option>`).join('')}
            </select>
          </div>
          <div class="col-6">
            <label title="Scheduler pause before the next step (other protocols may run)">next_step_after</label>
            <input
              type="text"
              data-action="step-wait-after"
              title="Scheduler pause before the next step (other protocols may run)"
              ${scopeAttrs}
              value="${escapeAttr(step.next_step_after ?? step.wait_after ?? '')}"
              placeholder="optional"
            />
          </div>
          ${renderTreeParams(step, { scope, stepIdx: idx, parentIdx })}
        </div>`}
      </div>`;
  }

  function renderBranchSteps(step, parentIdx, branchName) {
    const steps = getBranchSteps(step, branchName);
    return `
      <div class="branch-box stack">
        <div class="section-header">
          <h3 style="margin:0">${branchName === 'on_yes' ? `On Yes (${steps.length})` : `On No (${steps.length})`}</h3>
          <button class="btn small" data-action="branch-step-add" data-parent-idx="${parentIdx}" data-branch="${branchName}">Add Action Step</button>
        </div>
        ${steps.map((branchStep, idx) => renderTreeStep(branchStep, idx, branchName, parentIdx)).join('') || '<div class="inline-note">No branch steps</div>'}
      </div>`;
  }

  function renderConfirmationStep(step, idx) {
    const confirmation = step.confirmation || { question: '', on_yes: [], on_no: [] };
    const execLocation = String(confirmation.execution_location || 'current');
    const execOptions = executionLocationOptions();
    const collapsed = isStepCollapsed('top', idx, null);
    return `
      <div class="step-card ${collapsed ? 'collapsed' : ''}">
        <div class="step-header">
          <div>
            <div class="step-title">Step ${idx + 1} · Confirmation</div>
            <div class="step-summary">${escapeHtml(stepSummary(step))}</div>
          </div>
          <div class="btns">
            <button class="btn small ghost btn-toggle-icon" data-action="step-toggle-collapse" data-scope="top" data-step-idx="${idx}" title="${collapsed ? 'Expand' : 'Collapse'}">
              <span class="chevron-icon ${collapsed ? 'is-collapsed' : 'is-open'}" aria-hidden="true"></span>
            </button>
            <button class="btn small ghost" data-action="step-add-below" data-scope="top" data-step-idx="${idx}">+ Below</button>
            <button class="btn small ghost" data-action="step-duplicate" data-scope="top" data-step-idx="${idx}">Duplicate</button>
            <button class="btn small ghost" data-action="step-move-up" data-scope="top" data-step-idx="${idx}">↑</button>
            <button class="btn small ghost" data-action="step-move-down" data-scope="top" data-step-idx="${idx}">↓</button>
            <button class="btn small danger" data-action="step-delete" data-scope="top" data-step-idx="${idx}">Delete</button>
          </div>
        </div>
        ${collapsed ? '' : `
        <div class="row">
          <div class="col-8">
            <label>Question</label>
            <input type="text" data-action="confirm-question" data-step-idx="${idx}" value="${escapeAttr(confirmation.question || '')}" />
          </div>
          <div class="col-4">
            <label>execution_location</label>
            <select data-action="confirm-execution-location" data-step-idx="${idx}">
              ${execOptions.map((opt) => `<option value="${escapeAttr(opt)}" ${execLocation === opt ? 'selected' : ''}>${escapeHtml(opt)}</option>`).join('')}
            </select>
          </div>
          <div class="col-6">
            <label title="Scheduler pause before the next step (other protocols may run)">next_step_after (after confirmation branch)</label>
            <input
              type="text"
              data-action="step-wait-after"
              title="Scheduler pause before the next step (other protocols may run)"
              data-scope="top"
              data-step-idx="${idx}"
              value="${escapeAttr(step.next_step_after ?? step.wait_after ?? '')}"
              placeholder="optional"
            />
          </div>
          <div class="col-12 branch-columns">
            ${renderBranchSteps(step, idx, 'on_yes')}
            ${renderBranchSteps(step, idx, 'on_no')}
          </div>
        </div>`}
      </div>`;
  }

  function renderStepsSection(protocol) {
    const steps = currentSteps(protocol);
    return `
      <div class="section stack">
        <div class="section-header">
          <div>
            <h3>Action Steps</h3>
            <div class="inline-note">Build the protocol action flow. Nested confirmation is not supported.</div>
          </div>
          <div class="btns">
            <button class="btn small" data-action="steps-expand-all">Expand All</button>
            <button class="btn small" data-action="steps-collapse-all">Collapse All</button>
            <button class="btn small primary" data-action="step-add-tree">Add Action Step</button>
            <button class="btn small primary" data-action="step-add-confirmation">Add Confirmation</button>
          </div>
        </div>
        ${steps.map((step, idx) => ('confirmation' in step ? renderConfirmationStep(step, idx) : renderTreeStep(step, idx))).join('') || '<div class="inline-note">No steps defined.</div>'}
      </div>`;
  }

  function renderEditorHeader(protocolName) {
    const statusLabel = validationStatusLabel();
    const saveMeta = state.lastSavedAt && state.lastSaveInfo
      ? `Saved ${state.lastSavedAt} · Backup: ${state.lastSaveInfo.backup_path}`
      : state.lastSavedAt ? `Saved ${state.lastSavedAt}` : '';
    return `
      <div class="editor-sticky-header">
        <div class="editor-sticky-right">
          <div class="btns">
            <button class="btn btn-iconable" data-action="reload-bootstrap" title="Reload">
              <span class="btn-icon" aria-hidden="true">↻</span>
              <span class="btn-text">Reload</span>
            </button>
            <button class="btn btn-iconable" data-action="duplicate-protocol" title="Duplicate protocol">
              <span class="btn-icon" aria-hidden="true">⧉</span>
              <span class="btn-text">Duplicate</span>
            </button>
            <button class="btn danger btn-iconable" data-action="delete-protocol" title="Delete protocol">
              <span class="btn-icon" aria-hidden="true">✕</span>
              <span class="btn-text">Delete</span>
            </button>
            <button class="btn primary btn-iconable" data-action="save-config" title="Save">
              <span class="btn-icon mobile-only" aria-hidden="true">✓</span>
              <span class="btn-text">Save</span>
            </button>
          </div>
          ${saveMeta ? `<div class="inline-note editor-save-meta">${escapeHtml(saveMeta)}</div>` : ''}
        </div>
      </div>`;
  }

  function renderCollapsibleSection(key, title, desc, innerHtml) {
    const open = state.ui.sectionsOpen[key] !== false;
    return `
      <div class="section collapsible-section primary-section ${open ? '' : 'is-collapsed'}">
        <div class="section-header" data-action="section-toggle" data-section="${escapeAttr(key)}" role="button" tabindex="0" aria-expanded="${open ? 'true' : 'false'}">
          <div>
            <h3>${escapeHtml(title)}</h3>
            ${desc ? `<div class="inline-note">${escapeHtml(desc)}</div>` : ''}
          </div>
          <button class="btn small ghost btn-toggle-icon" data-action="section-toggle" data-section="${escapeAttr(key)}" title="${open ? 'Collapse section' : 'Expand section'}">
            <span class="chevron-icon ${open ? 'is-open' : 'is-collapsed'}" aria-hidden="true"></span>
          </button>
        </div>
        ${open ? innerHtml : ''}
      </div>`;
  }

  function renderOtherParametersSection() {
    const value = state.config?.person_init ?? '';
    const options = personInitOptions();
    return `
      <div class="form-grid">
        <div class="field">
          <label>person_init</label>
          <select data-action="person-init">
            ${options.map((opt) => {
              const label = opt === '' ? 'None' : opt;
              return `<option value="${escapeAttr(opt)}" ${value === opt ? 'selected' : ''}>${escapeHtml(label)}</option>`;
            }).join('')}
          </select>
          <div class="inline-note">Optional initial person landmark loaded into robot state at startup. Select None to disable.</div>
        </div>
      </div>`;
  }

  function renderEditorPanel() {
    const protocol = selectedProtocolObj();
    if (!protocol) {
      return `<div class="panel"><h2>Protocol Editor</h2><p class="muted">Select a GenericProtocol from the list or create a new one.</p></div>`;
    }
    ensureGenericShapes(protocol);
    const name = state.selectedProtocol;

    return `
      <div class="panel stack designer-editor-panel">
        ${renderEditorHeader(name)}

        <div class="section primary-section">
          <div class="section-header protocol-name-header">
            <h3>1. Protocol Name</h3>
            <div class="protocol-name-inline">
              <input type="text" data-action="protocol-rename" value="${escapeAttr(name)}" />
            </div>
          </div>
        </div>

        ${renderCollapsibleSection('triggers', '2. Triggers', 'Define when the protocol becomes eligible to run.', `
          ${renderTriggerBuilder(protocol)}
        `)}

        ${renderCollapsibleSection('actions', '3. Actions', 'Compose action steps and confirmation branches for this protocol.', renderStepsSection(protocol))}

        ${renderCollapsibleSection('reset', '4. Scheduling & Reset', 'Priority and reset pattern determine how often the protocol can run.', renderResetSection(protocol))}

        ${renderCollapsibleSection('other_params', '5. Other Parameters', 'Global house-yaml parameters that affect runtime behavior outside individual protocol definitions.', renderOtherParametersSection())}

        ${renderCollapsibleSection('yaml', '6. YAML Preview', 'Preview updates after save validation.', `
          <pre class="code-preview">${escapeHtml(state.yamlPreview || '')}</pre>
        `)}
      </div>`;
  }

  function renderProtocolDesigner() {
    els.routeView.innerHTML = `
      <div class="designer-layout">
        ${renderProtocolListPanel()}
        ${renderEditorPanel()}
      </div>`;
  }

  function escapeHtml(value) {
    return String(value ?? '')
      .replace(/&/g, '&amp;')
      .replace(/</g, '&lt;')
      .replace(/>/g, '&gt;')
      .replace(/"/g, '&quot;')
      .replace(/'/g, '&#039;');
  }
  const escapeAttr = escapeHtml;
  const capitalize = (v) => (v ? v.charAt(0).toUpperCase() + v.slice(1) : '');

  function applyBootstrapData(data) {
    state.config = data.config;
    state.metadata = data.metadata;
    state.yamlPath = data.yaml_path;
    state.yamlPreview = data.yaml_preview || '';
    const generics = getGenericProtocolNames();
    if (!state.selectedProtocol || !generics.includes(state.selectedProtocol)) {
      state.selectedProtocol = generics[0] || null;
    }
    markRecentProtocol(state.selectedProtocol);
    els.yamlPathLabel.textContent = state.yamlPath;
    setDirty(false);
    stopReloadPoll();
    clearFlash();
    updateStatusChip();
  }

  async function bootstrap() {
    state.loading = true;
    state.bootError = null;
    updateStatusChip();
    renderRoute();
    try {
      const data = await api('/api/dashboard/bootstrap');
      applyBootstrapData(data);
    } catch (err) {
      state.bootError = err.message;
      setFlash('error', err.message);
    } finally {
      state.loading = false;
      updateStatusChip();
      renderRoute();
      // Start home polling if we booted on the home route
      if (state.route === 'home') startHomePoll();
    }
  }

  function selectedStepsByScope(scope, stepIdx, parentIdx) {
    const protocol = selectedProtocolObj();
    const topSteps = currentSteps(protocol);
    if (scope === 'top') return topSteps;
    const parent = topSteps[parentIdx];
    if (!parent || !parent.confirmation) throw new Error('Invalid branch parent step');
    return getBranchSteps(parent, scope);
  }

  function parseMaybeNumber(text) {
    if (text === '' || text == null) return '';
    if (/^-?\d+$/.test(String(text))) return Number.parseInt(text, 10);
    if (/^-?\d+\.\d+$/.test(String(text))) return Number.parseFloat(text);
    return text;
  }

  function normalizeTreeParamsForStep(step) {
    const meta = metadataForTree(step.tree_name);
    const nextParams = {};
    for (const param of meta.params || []) {
      const existing = step.tree_params?.[param.name];
      if (existing !== undefined && existing !== '') nextParams[param.name] = existing;
      else if (param.required) nextParams[param.name] = makeDefaultParamValue(param);
    }
    if (step.tree_name === 'move_away') {
      if (nextParams.position_state_key === '' || nextParams.position_state_key == null) {
        nextParams.position_state_key = 'position';
      }
      if (nextParams.state_key === '' || nextParams.state_key == null) {
        nextParams.state_key = 'move_away';
      }
    }
    if ((meta.params || []).some((p) => p.name === 'execution_location')) {
      if (!nextParams.execution_location) nextParams.execution_location = 'current';
    }
    step.tree_params = nextParams;
  }

  function normalizeScalarEvent(evt) {
    const uiKind = eventUiKindForState(evt.state);
    if (uiKind === 'xy_proximity') return normalizeXYEvent(evt);
    delete evt.within_m;
    delete evt.point_xy;
    const kind = eventKindForState(evt.state, evt.value);
    if (kind === 'number') {
      evt.value = typeof evt.value === 'number' ? evt.value : 0;
      evt.op = evt.op || '=';
      delete evt.edge;
    } else if (kind === 'bool') {
      evt.value = evt.value === true;
      delete evt.op;
      if (evt.edge !== 'rising' && evt.edge !== 'falling') {
        delete evt.edge;
      } else if (evt.edge === 'rising') {
        evt.value = true;
      } else if (evt.edge === 'falling') {
        evt.value = false;
      }
    } else if (uiKind === 'location_name') {
      const opts = locationValueOptionsForState(evt.state);
      if (!opts.includes(evt.value)) evt.value = opts[0] || '';
      delete evt.op;
      delete evt.edge;
    } else {
      evt.value = (evt.value == null || typeof evt.value === 'object') ? '' : String(evt.value);
      delete evt.op;
      delete evt.edge;
    }
  }

  function normalizeXYEvent(evt) {
    delete evt.value;
    delete evt.op;
    delete evt.edge;
    evt.within_m = typeof evt.within_m === 'number' ? evt.within_m : 1.0;
    if (!Array.isArray(evt.point_xy) || evt.point_xy.length !== 2) evt.point_xy = [0.0, 0.0];
  }

  function ensureTriggers(protocol) {
    ensureGenericShapes(protocol);
    protocol.high_level.triggers = normalizeTriggerNode(protocol.high_level.triggers || {}, true);
    return protocol.high_level.triggers;
  }

  async function validateCurrentConfig() {
    const client = clientValidationForSelected();
    if (client.messages.length) {
      state.validation.status = 'error';
      state.validation.messages = client.messages;
      state.validation.lastValidatedAt = nowTimeLabel();
      setFlash('warn', 'Client-side validation found issues. Fix highlighted fields or run Validate to confirm backend errors.');
      renderRoute();
      return;
    }

    try {
      const data = await api('/api/dashboard/validate', {
        method: 'POST',
        body: JSON.stringify({ config: state.config }),
      });
      state.yamlPreview = data.yaml_preview || '';
      state.validation.status = 'ok';
      state.validation.messages = ['Schema validation passed.'];
      state.validation.lastValidatedAt = nowTimeLabel();
      setFlash('success', 'Validation passed.');
      renderRoute();
    } catch (err) {
      state.validation.status = 'error';
      state.validation.messages = [err.message];
      state.validation.lastValidatedAt = nowTimeLabel();
      setFlash('error', err.message);
      renderRoute();
    }
  }

  async function saveCurrentConfig() {
    try {
      const client = clientValidationForSelected();
      if (client.messages.length) {
        state.validation.status = 'error';
        state.validation.messages = client.messages;
        state.validation.lastValidatedAt = nowTimeLabel();
        setFlash('error', 'Cannot save: fix the highlighted fields first.');
        renderRoute();
        return;
      }

      const validateData = await api('/api/dashboard/validate', {
        method: 'POST',
        body: JSON.stringify({ config: state.config }),
      });
      state.yamlPreview = validateData.yaml_preview || state.yamlPreview;
      state.validation.status = 'ok';
      state.validation.messages = ['Schema validation passed.'];
      state.validation.lastValidatedAt = nowTimeLabel();

      const data = await api('/api/dashboard/save', {
        method: 'POST',
        body: JSON.stringify({ config: state.config, yaml_path: state.yamlPath }),
      });
      applyBootstrapData(data);
      state.lastSaveInfo = data.save || null;
      state.lastSavedAt = data.save?.saved_at ? new Date(data.save.saved_at).toLocaleTimeString() : nowTimeLabel();
      state.validation.status = 'ok';
      state.validation.messages = ['Saved and schema validation passed.'];
      state.validation.lastValidatedAt = nowTimeLabel();
      setFlash('success', `YAML saved successfully. Backup: ${data.save.backup_path}`);
      if (data.reload?.state === 'pending') {
        stopReloadPoll();
        state.reload.pollTimer = window.setTimeout(pollReloadStatus, 1500);
      } else if (data.reload?.state === 'error') {
        setFlash('error', `YAML saved successfully. Runtime YAML reload failed: ${data.reload.error || 'unknown error'}`);
      } else if (data.reload?.state === 'applied') {
        setFlash('success', 'Runtime YAML reloaded.');
      }
      renderRoute();
    } catch (err) {
      state.validation.status = 'error';
      state.validation.messages = [err.message];
      setFlash('error', err.message);
      renderRoute();
    }
  }

  function addProtocol() {
    const name = prompt('New GenericProtocol name (unique key):');
    if (!name) return;
    const trimmed = name.trim();
    if (!trimmed) return;
    if (getProtocols()[trimmed]) {
      setFlash('error', `Protocol '${trimmed}' already exists.`);
      return;
    }
    getProtocols()[trimmed] = {
      runner: 'GenericProtocol',
      high_level: { priority: 10, triggers: {}, reset_pattern: { type: 'eod' } },
      action: { steps: [makeDefaultTreeStep('play_text')] },
    };
    state.selectedProtocol = trimmed;
    markRecentProtocol(trimmed);
    setDirty(true);
    renderRoute();
  }

  function duplicateSelectedProtocol() {
    if (!state.selectedProtocol) return;
    const base = state.selectedProtocol;
    let proposed = `${base}_copy`;
    let i = 2;
    while (getProtocols()[proposed]) {
      proposed = `${base}_copy_${i++}`;
    }
    const entered = prompt('Duplicate protocol as:', proposed);
    if (!entered) return;
    const newName = entered.trim();
    if (!newName) return;
    if (getProtocols()[newName]) {
      setFlash('error', `Protocol '${newName}' already exists.`);
      return;
    }
    getProtocols()[newName] = deepClone(getProtocols()[base]);
    state.selectedProtocol = newName;
    markRecentProtocol(newName);
    setDirty(true);
    renderRoute();
  }

  function deleteSelectedProtocol() {
    if (!state.selectedProtocol) return;
    const name = state.selectedProtocol;
    if (!confirm(`Delete protocol '${name}'?`)) return;
    delete getProtocols()[name];
    const names = getGenericProtocolNames();
    state.selectedProtocol = names[0] || null;
    markRecentProtocol(state.selectedProtocol);
    setDirty(true);
    renderRoute();
  }

  function mutateStepArray(action, btn) {
    const scope = btn.dataset.scope;
    const stepIdx = Number(btn.dataset.stepIdx);
    const parentIdx = btn.dataset.parentIdx !== undefined ? Number(btn.dataset.parentIdx) : null;
    const arr = selectedStepsByScope(scope, stepIdx, parentIdx);
    if (!Array.isArray(arr)) return false;
    if (action === 'step-delete') arr.splice(stepIdx, 1);
    else if (action === 'step-move-up' && stepIdx > 0) [arr[stepIdx - 1], arr[stepIdx]] = [arr[stepIdx], arr[stepIdx - 1]];
    else if (action === 'step-move-down' && stepIdx < arr.length - 1) [arr[stepIdx + 1], arr[stepIdx]] = [arr[stepIdx], arr[stepIdx + 1]];
    else if (action === 'step-move-top' && stepIdx > 0) arr.unshift(arr.splice(stepIdx, 1)[0]);
    else if (action === 'step-move-bottom' && stepIdx < arr.length - 1) arr.push(arr.splice(stepIdx, 1)[0]);
    else if (action === 'step-duplicate') arr.splice(stepIdx + 1, 0, deepClone(arr[stepIdx]));
    else if (action === 'step-add-below') {
      const template = arr[stepIdx]?.confirmation ? makeDefaultConfirmationStep() : makeDefaultTreeStep('play_text');
      arr.splice(stepIdx + 1, 0, template);
    } else return false;
    return true;
  }

  function handleClick(event) {
    const btn = event.target.closest('[data-action]');
    if (!btn) return;
    const action = btn.dataset.action;
    try {
      if (action === 'select-protocol') {
        const nextName = btn.dataset.name;
        if (state.dirty && state.selectedProtocol && nextName !== state.selectedProtocol) {
          const proceed = confirm('You have unsaved changes. Switch protocol anyway?');
          if (!proceed) return;
        }
        state.selectedProtocol = nextName;
        markRecentProtocol(nextName);
        renderRoute();
        return;
      }
      if (action === 'section-toggle') {
        const key = btn.dataset.section;
        state.ui.sectionsOpen[key] = !(state.ui.sectionsOpen[key] !== false);
        renderRoute();
        return;
      }
      if (action === 'toggle-special-panel') {
        state.ui.specialCollapsed = !state.ui.specialCollapsed;
        renderRoute();
        return;
      }
      if (action === 'reload-bootstrap') {
        bootstrap();
        return;
      }
      if (action === 'flash-close') {
        clearFlash();
        return;
      }
      if (action === 'validate-config') {
        return;
      }
      if (action === 'save-config') {
        saveCurrentConfig();
        return;
      }
      if (action === 'duplicate-protocol') {
        duplicateSelectedProtocol();
        return;
      }
      if (action === 'add-protocol') {
        addProtocol();
        return;
      }
      if (action === 'delete-protocol') {
        deleteSelectedProtocol();
        return;
      }

      const protocol = selectedProtocolObj();
      if (!protocol) return;
      ensureGenericShapes(protocol);

      if (action === 'trigger-add-node') {
        addTriggerNode(protocol, btn.dataset.path || '', btn.dataset.kind || 'event');
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'trigger-remove-node') {
        removeTriggerNode(protocol, btn.dataset.path || '');
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'trigger-time-meridiem') {
        const node = getTriggerNodeAtPath(protocol, btn.dataset.path || '');
        const time = node.time ||= { from: '', to: '', day: [] };
        const field = btn.dataset.field;
        time[field] = setTimePart(time[field], 'meridiem', btn.dataset.value || 'AM');
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'step-add-tree') {
        currentSteps(protocol).push(makeDefaultTreeStep('play_text'));
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'step-add-confirmation') {
        currentSteps(protocol).push(makeDefaultConfirmationStep());
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'branch-step-add') {
        const parentIdx = Number(btn.dataset.parentIdx);
        const branch = btn.dataset.branch;
        getBranchSteps(currentSteps(protocol)[parentIdx], branch).push(makeDefaultTreeStep('play_text'));
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'steps-expand-all' || action === 'steps-collapse-all') {
        if (action === 'steps-expand-all') state.ui.stepCollapsed = {};
        else {
          const map = {};
          currentSteps(protocol).forEach((_, idx) => { map[stepKey('top', idx)] = true; });
          state.ui.stepCollapsed = map;
        }
        renderRoute();
        return;
      }
      if (action === 'step-toggle-collapse') {
        const scope = btn.dataset.scope;
        const stepIdx = Number(btn.dataset.stepIdx);
        const parentIdx = btn.dataset.parentIdx !== undefined ? Number(btn.dataset.parentIdx) : null;
        setStepCollapsed(scope, stepIdx, !isStepCollapsed(scope, stepIdx, parentIdx), parentIdx);
        renderRoute();
        return;
      }
      if ([
        'step-delete', 'step-move-up', 'step-move-down', 'step-move-top', 'step-move-bottom', 'step-duplicate', 'step-add-below'
      ].includes(action)) {
        if (mutateStepArray(action, btn)) {
          setDirty(true);
          renderRoute();
        }
        return;
      }
    } catch (err) {
      setFlash('error', err.message);
      console.error(err);
    }
  }

  function handleInput(event) {
    const target = event.target;
    const action = target.dataset.action;
    if (!action) return;

    if (action === 'list-search') {
      const selStart = target.selectionStart;
      const selEnd = target.selectionEnd;
      state.ui.listSearch = target.value;
      renderRoute();
      const next = els.routeView.querySelector('input[data-action="list-search"]');
      if (next) {
        next.focus();
        if (selStart != null && selEnd != null) {
          next.setSelectionRange(selStart, selEnd);
        }
      }
      return;
    }
  }

  function handleChange(event) {
    const target = event.target;
    const action = target.dataset.action;
    if (!action) return;
    const protocol = selectedProtocolObj();
    try {
      if (action === 'list-sort') {
        state.ui.listSort = target.value;
        renderRoute();
        return;
      }
      if (action === 'list-filter') {
        state.ui.filters[target.dataset.key] = target.checked;
        renderRoute();
        return;
      }
      if (action === 'protocol-rename') {
        const oldName = state.selectedProtocol;
        const newName = target.value.trim();
        if (!oldName || !newName || oldName === newName) return;
        if (getProtocols()[newName]) throw new Error(`Protocol '${newName}' already exists.`);
        const val = getProtocols()[oldName];
        delete getProtocols()[oldName];
        getProtocols()[newName] = val;
        state.selectedProtocol = newName;
        markRecentProtocol(newName);
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'person-init') {
        state.config.person_init = target.value || null;
        setDirty(true);
        return;
      }
      if (!protocol) return;
      ensureGenericShapes(protocol);

      if (action === 'trigger-group-mode') {
        setTriggerGroupMode(protocol, target.dataset.path || '', target.value);
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'trigger-location-toggle') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const arr = node.permissible_locations ||= [];
        const s = new Set(arr);
        if (target.checked) s.add(target.dataset.location); else s.delete(target.dataset.location);
        node.permissible_locations = [...s];
        setDirty(true);
        return;
      }
      if (action === 'trigger-time-day-toggle') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const time = node.time ||= { from: '', to: '', day: [] };
        const days = new Set(time.day || []);
        if (target.checked) days.add(target.dataset.day); else days.delete(target.dataset.day);
        time.day = [...days];
        setDirty(true);
        return;
      }
      if (action === 'priority') {
        protocol.high_level.priority = Number(target.value || 0);
        setDirty(true);
        return;
      }
      if (action === 'reset-type') {
        protocol.high_level.reset_pattern ||= {};
        protocol.high_level.reset_pattern.type = target.value;
        if (target.value !== 'periodic') {
          delete protocol.high_level.reset_pattern.hours;
          delete protocol.high_level.reset_pattern.minutes;
        }
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'reset-periodic-hours' || action === 'reset-periodic-minutes') {
        protocol.high_level.reset_pattern ||= { type: 'periodic' };
        const key = action.endsWith('hours') ? 'hours' : 'minutes';
        if (target.value === '') delete protocol.high_level.reset_pattern[key];
        else protocol.high_level.reset_pattern[key] = Number(target.value);
        setDirty(true);
        return;
      }
      if (action === 'trigger-event-state') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const evt = node.event ||= makeDefaultEventTrigger();
        evt.state = target.value;
        normalizeScalarEvent(evt);
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'trigger-event-op') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        node.event.op = target.value;
        setDirty(true);
        return;
      }
      if (action === 'trigger-event-within-m') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        node.event.within_m = Number(target.value || 0);
        setDirty(true);
        return;
      }
      if (action === 'trigger-event-point-xy-csv') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const evt = node.event;
        const parts = String(target.value || '').split(',').map((s) => s.trim()).filter(Boolean);
        if (parts.length === 2) {
          const x = Number(parts[0]);
          const y = Number(parts[1]);
          if (!Number.isNaN(x) && !Number.isNaN(y)) evt.point_xy = [x, y];
        }
        setDirty(true);
        return;
      }
      if (action === 'trigger-event-value') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const evt = node.event;
        const type = target.dataset.type;
        if (type === 'bool') {
          if (target.value === 'rising') {
            evt.value = true;
            evt.edge = 'rising';
          } else if (target.value === 'falling') {
            evt.value = false;
            evt.edge = 'falling';
          } else {
            evt.value = target.value === 'true';
            delete evt.edge;
          }
        }
        else if (type === 'number') evt.value = Number(target.value || 0);
        else evt.value = target.value;
        if (type !== 'number') delete evt.op;
        setDirty(true);
        return;
      }
      if (action === 'trigger-time-part') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const time = node.time ||= { from: '', to: '', day: [] };
        const field = target.dataset.field;
        const part = target.dataset.part;
        time[field] = setTimePart(time[field], part, target.value);
        setDirty(true);
        return;
      }
      if (action === 'trigger-protocol-completion-protocol') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const rule = node.protocol_completion ||= makeDefaultProtocolCompletionTrigger();
        rule.protocol = target.value;
        setDirty(true);
        return;
      }
      if (action === 'trigger-protocol-completion-after') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const rule = node.protocol_completion ||= makeDefaultProtocolCompletionTrigger();
        if (target.value === '') delete rule.after_seconds;
        else rule.after_seconds = Number(target.value);
        setDirty(true);
        return;
      }
      if (action === 'trigger-protocol-completion-within') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const rule = node.protocol_completion ||= makeDefaultProtocolCompletionTrigger();
        rule.within_seconds = Number(target.value || 0);
        setDirty(true);
        return;
      }
      if (action === 'trigger-protocol-completion-status') {
        const node = getTriggerNodeAtPath(protocol, target.dataset.path || '');
        const rule = node.protocol_completion ||= makeDefaultProtocolCompletionTrigger();
        const statuses = new Set(Array.isArray(rule.statuses) ? rule.statuses : []);
        const status = target.dataset.status;
        if (target.checked) statuses.add(status);
        else statuses.delete(status);
        rule.statuses = [...statuses];
        setDirty(true);
        return;
      }
      if (action === 'step-tree-name') {
        const arr = selectedStepsByScope(
          target.dataset.scope,
          Number(target.dataset.stepIdx),
          target.dataset.parentIdx !== undefined ? Number(target.dataset.parentIdx) : null,
        );
        arr[Number(target.dataset.stepIdx)].tree_name = target.value;
        normalizeTreeParamsForStep(arr[Number(target.dataset.stepIdx)]);
        setDirty(true);
        renderRoute();
        return;
      }
      if (action === 'step-tree-param') {
        const scope = target.dataset.scope;
        const stepIdx = Number(target.dataset.stepIdx);
        const parentIdx = target.dataset.parentIdx !== undefined ? Number(target.dataset.parentIdx) : null;
        const arr = selectedStepsByScope(scope, stepIdx, parentIdx);
        const step = arr[stepIdx];
        step.tree_params ||= {};
        const paramName = target.dataset.param;
        const meta = metadataForTree(step.tree_name);
        const spec = (meta.params || []).find((p) => p.name === paramName);
        let value;
        if (spec?.kind === 'integer') {
          value = target.value === '' ? '' : Number.parseInt(target.value, 10);
        } else if (spec?.kind === 'number') {
          value = target.value === '' ? '' : Number.parseFloat(target.value);
        } else if (spec?.kind === 'audio_asset' || spec?.kind === 'video_asset') {
          value = target.value || '';
        } else if (spec?.kind === 'execution_location') {
          value = target.value || 'current';
        } else {
          value = parseMaybeNumber(target.value);
        }
        if (value === '' && !spec?.required) delete step.tree_params[paramName];
        else step.tree_params[paramName] = value;
        setDirty(true);
        return;
      }
      if (action === 'step-wait-after') {
        const scope = target.dataset.scope;
        const stepIdx = Number(target.dataset.stepIdx);
        const parentIdx = target.dataset.parentIdx !== undefined ? Number(target.dataset.parentIdx) : null;
        const arr = selectedStepsByScope(scope, stepIdx, parentIdx);
        if (target.value === '') delete arr[stepIdx].next_step_after;
        else arr[stepIdx].next_step_after = parseMaybeNumber(target.value);
        delete arr[stepIdx].wait_after; // migrate old key if present
        setDirty(true);
        return;
      }
      if (action === 'confirm-question') {
        currentSteps(protocol)[Number(target.dataset.stepIdx)].confirmation.question = target.value;
        setDirty(true);
        return;
      }
      if (action === 'confirm-execution-location') {
        const confirmation = currentSteps(protocol)[Number(target.dataset.stepIdx)].confirmation;
        confirmation.execution_location = target.value || 'current';
        setDirty(true);
        return;
      }
    } catch (err) {
      setFlash('error', err.message);
      console.error(err);
    }
  }

  // ======================================================================
  // Home Dashboard
  // ======================================================================

  function startHomePoll() {
    stopHomePoll();
    pollHomeData(); // immediate first fetch
    state.home.pollTimer = setInterval(pollHomeData, 2000);
  }

  function stopHomePoll() {
    if (state.home.pollTimer) {
      clearInterval(state.home.pollTimer);
      state.home.pollTimer = null;
    }
  }

  async function pollHomeData() {
    try {
      const [rsRes, psRes, sumRes] = await Promise.all([
        fetch('/api/robot-state').then((r) => r.json()).catch(() => null),
        fetch('/api/protocol-state').then((r) => r.json()).catch(() => null),
        fetch('/api/protocol-summary').then((r) => r.json()).catch(() => null),
      ]);
      state.home.robotState = rsRes?.ok ? rsRes.state : null;
      state.home.protocolStates = psRes?.ok ? (psRes.states || []) : [];
      state.home.summary = sumRes?.ok ? sumRes : null;
      state.home.error = null;
    } catch (err) {
      state.home.error = String(err);
    }
    if (state.route === 'home') renderHomeDashboard();
  }

  function renderHomeDashboard() {
    const rs = state.home.robotState;
    const ps = state.home.protocolStates;
    const sum = state.home.summary;

    els.routeView.innerHTML = `
      <div class="home-grid">
        ${renderPowerCard(rs)}
        ${renderRobotStatusCard(rs)}
        ${renderProtocolActivityCard(ps)}
        ${renderTodaySummaryCard(sum)}
        ${renderSensorEventsCard(rs)}
      </div>
    `;
  }

  function renderPowerCard(rs) {
    const pct = rs?.battery_percent;
    const hasPct = pct != null;
    const pctNum = hasPct ? Number(pct) : 0;
    const deg = hasPct ? Math.round((pctNum / 100) * 360) : 0;
    const color = pctNum > 50 ? 'var(--ok)' : pctNum > 20 ? 'var(--warn)' : 'var(--danger)';
    const charging = rs?.charging;
    const v36 = rs?.voltage_36v;
    const v12 = rs?.voltage_12v;

    return `
      <div class="dash-card">
        <div class="dash-card-header"><h3>Power & Battery</h3><span class="dash-card-icon">⚡</span></div>
        <div class="battery-gauge-wrap">
          <div class="battery-gauge" style="--gauge-deg:${deg}deg; --gauge-color:${color}">
            <span class="battery-gauge-value">${hasPct ? pctNum : '—'}<small>${hasPct ? '%' : ''}</small></span>
          </div>
          <div class="battery-gauge-label">Battery</div>
        </div>
        <div class="charging-bar ${charging === true ? 'on' : charging === false ? 'off' : 'unknown'}">
          <span class="charging-dot"></span>
          ${charging === true ? '⚡ Charging' : charging === false ? 'On Battery' : 'Status Unknown'}
        </div>
        <div class="voltage-grid">
          <div class="voltage-item">
            <div class="volt-label">36 V Bus</div>
            <div class="volt-value">${v36 != null ? Number(v36).toFixed(2) : '—'}</div>
          </div>
          <div class="voltage-item">
            <div class="volt-label">12 V Bus</div>
            <div class="volt-value">${v12 != null ? Number(v12).toFixed(2) : '—'}</div>
          </div>
        </div>
      </div>`;
  }

  function renderRobotStatusCard(rs) {
    const kv = [
      ['Robot Location', rs?.robot_location],
      ['Person Location', rs?.person_location],
      ['Position Mode', rs?.position],
    ];
    const tilt = rs?.over_tilt === true;
    const xy = rs?.robot_location_xy;
    const xyStr = Array.isArray(xy) && xy.length === 2 ? `(${Number(xy[0]).toFixed(2)}, ${Number(xy[1]).toFixed(2)})` : null;

    return `
      <div class="dash-card">
        <div class="dash-card-header"><h3>Robot Status</h3><span class="dash-card-icon">🤖</span></div>
        ${tilt ? '<div class="alert-banner danger">⚠ Over-Tilt Alert! Robot may have tipped.</div>' : ''}
        <div class="status-kv">
          ${kv.map(([label, val]) => `
            <div class="status-kv-row">
              <span class="status-kv-label">${escapeHtml(label)}</span>
              <span class="status-kv-value">${val != null ? escapeHtml(val) : '<span class="muted">—</span>'}</span>
            </div>`).join('')}
          ${xyStr ? `
            <div class="status-kv-row">
              <span class="status-kv-label">Coordinates</span>
              <span class="status-kv-value" style="font-family:monospace">${escapeHtml(xyStr)}</span>
            </div>` : ''}
        </div>
      </div>`;
  }

  function renderProtocolActivityCard(ps) {
    const counts = { running: 0, idle: 0, cooldown: 0, waiting: 0 };
    for (const p of ps) {
      const s = (p.state || '').toLowerCase();
      if (s in counts) counts[s]++;
    }
    const running = ps.filter((p) => (p.state || '').toLowerCase() === 'running');

    let tableHtml = '';
    if (running.length) {
      const rows = running.map((p) => {
        const done = p.completed_step || 0;
        const total = p.total_steps || 1;
        const pct = Math.min(100, Math.round((done / total) * 100));
        const startTime = p.started_at ? new Date(p.started_at).toLocaleTimeString() : '—';
        return `
          <tr>
            <td>${escapeHtml(p.protocol)}</td>
            <td><div class="mini-progress"><div class="mini-progress-bar" style="width:${pct}%"></div></div></td>
            <td>${done}/${total}</td>
            <td>${startTime}</td>
          </tr>`;
      }).join('');
      tableHtml = `
        <table class="running-table">
          <thead><tr><th>Protocol</th><th>Progress</th><th>Step</th><th>Started</th></tr></thead>
          <tbody>${rows}</tbody>
        </table>`;
    } else {
      tableHtml = '<div class="dash-no-data">No protocols currently running</div>';
    }

    return `
      <div class="dash-card">
        <div class="dash-card-header"><h3>Protocol Activity</h3><span class="dash-card-icon">▶</span></div>
        <div class="proto-counts">
          <div class="proto-count-tile running"><div class="count-num">${counts.running}</div><div class="count-label">Running</div></div>
          <div class="proto-count-tile idle"><div class="count-num">${counts.idle}</div><div class="count-label">Idle</div></div>
          <div class="proto-count-tile cooldown"><div class="count-num">${counts.cooldown}</div><div class="count-label">Cooldown</div></div>
          <div class="proto-count-tile waiting"><div class="count-num">${counts.waiting}</div><div class="count-label">Waiting</div></div>
        </div>
        ${tableHtml}
      </div>`;
  }

  function renderTodaySummaryCard(sum) {
    const completed = sum?.completed ?? '—';
    const failed = sum?.failed ?? '—';
    const preempted = sum?.preempted ?? '—';
    const total = sum?.total ?? '—';
    const dateLabel = sum?.date ? sum.date : 'Today';

    return `
      <div class="dash-card">
        <div class="dash-card-header"><h3>Today's Runs</h3><span class="dash-card-icon">📊</span></div>
        <div class="summary-counts">
          <div class="summary-tile completed"><div class="sum-num">${completed}</div><div class="sum-label">Completed</div></div>
          <div class="summary-tile failed"><div class="sum-num">${failed}</div><div class="sum-label">Failed</div></div>
          <div class="summary-tile preempted"><div class="sum-num">${preempted}</div><div class="sum-label">Preempted</div></div>
          <div class="summary-tile total"><div class="sum-num">${total}</div><div class="sum-label">Total</div></div>
        </div>
        <div class="dash-no-data" style="font-size:11px; margin-top:-4px">${escapeHtml(dateLabel)}</div>
      </div>`;
  }

  function renderSensorEventsCard(rs) {
    const sensors = [
      ['Coffee', rs?.coffee],
      ['Bathroom', rs?.bathroom_sensor],
      ['Start Exercise', rs?.start_exercise],
      ['Stop Exercise', rs?.stop_exercise],
    ];

    return `
      <div class="dash-card">
        <div class="dash-card-header"><h3>Sensor Events</h3><span class="dash-card-icon">📡</span></div>
        <div class="sensor-grid">
          ${sensors.map(([label, val]) => `
            <div class="sensor-dot-row">
              <span class="sensor-dot ${val === true ? 'active' : ''}"></span>
              <span class="sensor-dot-label">${escapeHtml(label)}</span>
            </div>`).join('')}
        </div>
      </div>`;
  }

  // ======================================================================
  // Protocol History page
  // ======================================================================

  let _historyEventSource = null;

  function connectSSE() {
    const h = state.history;
    // SSE is only meaningful for single-day view
    if (h.mode !== 'single' || !h.date) return;

    // Close any existing connection
    disconnectSSE();

    const url = `/api/protocol-events?date=${encodeURIComponent(h.date)}`;
    _historyEventSource = new EventSource(url);

    _historyEventSource.addEventListener('state', (e) => {
      try {
        if (h.mode !== 'single') return;
        const data = JSON.parse(e.data);
        h.states = data.states || [];
        if (state.route === 'protocol-history') renderProtocolHistory();
      } catch (_) { /* ignore parse errors */ }
    });

    _historyEventSource.addEventListener('log', (e) => {
      try {
        if (h.mode !== 'single') return;
        const data = JSON.parse(e.data);
        // Only update if the date matches what we're viewing
        if (data.date === h.date) {
          h.entries = data.entries || [];
          h.page = 1;
          if (state.route === 'protocol-history') renderProtocolHistory();
        }
      } catch (_) { /* ignore parse errors */ }
    });

    _historyEventSource.onerror = () => {
      // EventSource auto-reconnects; nothing to do
    };
  }

  function disconnectSSE() {
    if (_historyEventSource) {
      _historyEventSource.close();
      _historyEventSource = null;
    }
  }

  async function fetchHistory() {
    const h = state.history;
    h.loading = true;
    h.error = null;
    renderProtocolHistory();
    try {
      const params = new URLSearchParams();
      // All-dates mode should expose full run history, not compacted sessions.
      params.set('view', h.mode === 'all' ? 'all_runs' : 'compact');
      if (h.mode === 'single') {
        params.set('date', h.date);
      } else if (h.mode === 'range') {
        params.set('date_from', h.dateFrom);
        params.set('date_to', h.dateTo);
      } else if (h.mode === 'all') {
        params.set('all', 'true');
      }

      // Fetch history and state independently so history filtering still works
      // even if protocol-state endpoint fails.
      const [histRes, stateRes] = await Promise.allSettled([
        api(`/api/protocol-history?${params}`),
        api('/api/protocol-state'),
      ]);

      if (histRes.status === 'fulfilled') {
        const data = histRes.value;
        h.entries = data.entries || [];
        h.page = 1;

        // Keep local controls aligned with normalized backend query.
        const q = data.query || {};
        if (q.mode === 'single' && q.date) h.date = q.date;
        if (q.mode === 'range') {
          if (q.date_from) h.dateFrom = q.date_from;
          if (q.date_to) h.dateTo = q.date_to;
        }
      } else {
        // Avoid stale history rows when a new query fails.
        h.entries = [];
        h.error = histRes.reason?.message || 'Failed to load protocol history.';
      }

      if (stateRes.status === 'fulfilled') {
        h.states = stateRes.value.states || [];
      } else if (!h.error) {
        h.error = stateRes.reason?.message || 'Failed to load protocol state.';
      }
    } finally {
      h.loading = false;
      renderProtocolHistory();
      // Keep SSE only for single-day mode.
      if (h.mode === 'single') connectSSE();
      else disconnectSSE();
    }
  }

  function shortProtoName(full) {
    if (!full) return '';
    return full.includes('.') ? full.split('.').slice(1).join('.') : full;
  }

  function fmtTime(iso) {
    if (!iso) return '-';
    const t = iso.split('T')[1];
    return t ? t.slice(0, 8) : iso;
  }

  function fmtDate(entry) {
    if (!entry) return '-';
    if (entry.date) return entry.date;
    const iso = entry.start_time || entry.end_time || '';
    const d = iso.split('T')[0];
    return d || '-';
  }

  function durationStr(start, end) {
    if (!start || !end) return '-';
    const ms = new Date(end) - new Date(start);
    if (ms < 0) return '-';
    const s = Math.round(ms / 1000);
    if (s < 60) return `${s}s`;
    const m = Math.floor(s / 60);
    const rem = s % 60;
    return `${m}m ${rem}s`;
  }

  function historyStatusBadge(status) {
    const cls = status === 'completed' ? 'badge-ok'
      : status === 'failed' ? 'badge-danger'
      : status === 'preempted' ? 'badge-warn'
      : status === 'yielded' ? 'badge-info'
      : 'badge-muted';
    return `<span class="hist-badge ${cls}">${escapeHtml(status)}</span>`;
  }

  function stateStateBadge(st) {
    const cls = st === 'running' ? 'badge-primary'
      : st === 'cooldown' ? 'badge-warn'
      : st === 'waiting' ? 'badge-info'
      : 'badge-muted';
    return `<span class="hist-badge ${cls}">${escapeHtml(st)}</span>`;
  }

  function getFilteredHistoryEntries() {
    let entries = state.history.entries;
    const sf = state.history.statusFilter.toLowerCase();
    const pf = state.history.protocolFilter.toLowerCase();
    if (sf) entries = entries.filter(e => (e.status || '').toLowerCase() === sf);
    if (pf) entries = entries.filter(e => (e.protocol || '').toLowerCase().includes(pf));
    return entries;
  }

  function getHistoryPage(entries) {
    const h = state.history;
    const pageSize = h.pageSize || 20;
    const total = entries.length;
    const totalPages = Math.max(1, Math.ceil(total / pageSize));
    const page = Math.min(Math.max(1, h.page || 1), totalPages);
    h.page = page;
    const start = (page - 1) * pageSize;
    const pageEntries = entries.slice(start, start + pageSize);
    return {
      pageEntries,
      page,
      pageSize,
      total,
      totalPages,
      hasPrev: page > 1,
      hasNext: page < totalPages,
    };
  }

  function historyRangeLabel() {
    const h = state.history;
    if (h.mode === 'all') return 'All Dates';
    if (h.mode === 'range') return `${h.dateFrom} to ${h.dateTo}`;
    return h.date;
  }

  function renderProtocolHistory() {
    const h = state.history;

    if (h.loading) {
      els.routeView.innerHTML = '<div class="blank-card">Loading protocol history\u2026</div>';
      return;
    }

    const filtered = getFilteredHistoryEntries().slice().reverse();
    const pageInfo = getHistoryPage(filtered);

    // Unique protocol names for filter dropdown
    const uniqueProtos = [...new Set(h.entries.map(e => shortProtoName(e.protocol)))].sort();

    // --- States card ---
    const statesHtml = h.states.length ? h.states.map(s => {
      const canEdit = s.state !== 'idle' && s.state !== 'running';
      const actionCell = canEdit
        ? `<button class="btn btn-sm btn-outline hist-action-btn" data-action="hist-set-idle" data-protocol="${escapeAttr(s.protocol)}" title="Reset to idle">Set Idle</button>`
        : `<span class="muted">–</span>`;
      const progressCell = (s.total_steps != null && s.total_steps > 0)
        ? `<span class="hist-badge badge-primary">${s.completed_step || 0} / ${s.total_steps}</span>`
        : '<span class="muted">–</span>';
      return `
      <tr>
        <td class="hist-cell">${escapeHtml(shortProtoName(s.protocol))}</td>
        <td class="hist-cell">${stateStateBadge(s.state)}</td>
        <td class="hist-cell">${progressCell}</td>
        <td class="hist-cell mono">${fmtTime(s.started_at)}</td>
        <td class="hist-cell mono">${fmtTime(s.eligible_at)}</td>
        <td class="hist-cell mono">${fmtTime(s.last_completed)}</td>
        <td class="hist-cell">${s.last_status ? historyStatusBadge(s.last_status) : '<span class="muted">-</span>'}</td>
        <td class="hist-cell hist-actions-cell">${actionCell}</td>
      </tr>`;
    }).join('')
      : '<tr><td colspan="8" class="hist-cell muted">No protocol state tracked yet.</td></tr>';

    // --- Log table ---
    const showDateColumn = h.mode !== 'single';
    const emptyScope = h.mode === 'single' ? 'for this date' : (h.mode === 'range' ? 'for this date range' : 'for all dates');
    const logRows = pageInfo.pageEntries.length ? pageInfo.pageEntries.map(e => `
      <tr>
        <td class="hist-cell">${escapeHtml(shortProtoName(e.protocol))}</td>
        <td class="hist-cell">${historyStatusBadge(e.status)}</td>
        ${showDateColumn ? `<td class="hist-cell mono">${escapeHtml(fmtDate(e))}</td>` : ''}
        <td class="hist-cell mono">${fmtTime(e.start_time)}</td>
        <td class="hist-cell mono">${fmtTime(e.end_time)}</td>
        <td class="hist-cell mono">${durationStr(e.start_time, e.end_time)}</td>
        <td class="hist-cell">${e.failure_reason ? escapeHtml(e.failure_reason) : '<span class="muted">-</span>'}</td>
        <td class="hist-cell">${e.detail ? escapeHtml(e.detail) : '<span class="muted">-</span>'}</td>
      </tr>`).join('')
      : `<tr><td colspan="${showDateColumn ? 8 : 7}" class="hist-cell muted">No protocol runs found${h.entries.length ? ' matching filters' : ` ${emptyScope}` }.</td></tr>`;

    const protoOptions = uniqueProtos.map(p =>
      `<option value="${escapeAttr(p)}" ${p.toLowerCase() === h.protocolFilter.toLowerCase() ? 'selected' : ''}>${escapeHtml(p)}</option>`
    ).join('');

    const modeFields = `
      <div class="hist-filter-group">
        <label class="hist-label" for="histMode">Range</label>
        <select id="histMode" class="hist-input" data-action="hist-mode">
          <option value="single" ${h.mode === 'single' ? 'selected' : ''}>Single Day</option>
          <option value="range" ${h.mode === 'range' ? 'selected' : ''}>Date Range</option>
          <option value="all" ${h.mode === 'all' ? 'selected' : ''}>All</option>
        </select>
      </div>
      ${h.mode === 'single' ? `
      <div class="hist-filter-group">
        <label class="hist-label" for="histDate">Date</label>
        <input type="date" id="histDate" class="hist-input" value="${escapeAttr(h.date)}"
               data-action="hist-date" />
      </div>` : ''}
      ${h.mode === 'range' ? `
      <div class="hist-filter-group">
        <label class="hist-label" for="histDateFrom">From</label>
        <input type="date" id="histDateFrom" class="hist-input" value="${escapeAttr(h.dateFrom)}"
               data-action="hist-date-from" />
      </div>
      <div class="hist-filter-group">
        <label class="hist-label" for="histDateTo">To</label>
        <input type="date" id="histDateTo" class="hist-input" value="${escapeAttr(h.dateTo)}"
               data-action="hist-date-to" />
      </div>` : ''}
    `;

    els.routeView.innerHTML = `
      <div class="hist-page">
        <!-- Filters bar -->
        <div class="hist-filters panel">
          ${modeFields}
          <div class="hist-filter-group">
            <label class="hist-label" for="histStatus">Status</label>
            <select id="histStatus" class="hist-input" data-action="hist-status">
              <option value="">All</option>
              <option value="completed" ${h.statusFilter === 'completed' ? 'selected' : ''}>Completed</option>
              <option value="failed" ${h.statusFilter === 'failed' ? 'selected' : ''}>Failed</option>
              <option value="preempted" ${h.statusFilter === 'preempted' ? 'selected' : ''}>Preempted</option>
            </select>
          </div>
          <div class="hist-filter-group">
            <label class="hist-label" for="histProtocol">Protocol</label>
            <select id="histProtocol" class="hist-input" data-action="hist-protocol">
              <option value="">All</option>
              ${protoOptions}
            </select>
          </div>
          <div class="hist-filter-group hist-filter-actions">
            <button class="btn btn-sm" data-action="hist-refresh" title="Refresh">Refresh</button>
            <button class="btn btn-sm btn-outline" data-action="hist-clear-filters" title="Clear filters">Clear</button>
          </div>
          <div class="hist-filter-count">
            <span class="muted">${filtered.length} of ${h.entries.length} runs</span>
          </div>
        </div>

        ${h.error ? `<div class="flash error">${escapeHtml(h.error)}</div>` : ''}

        <!-- Live Protocol State -->
        <div class="panel hist-section">
          <h3 class="hist-section-title">Live Protocol State</h3>
          <div class="hist-table-wrap">
            <table class="hist-table">
              <thead>
                <tr>
                  <th class="hist-th">Protocol</th>
                  <th class="hist-th">State</th>
                  <th class="hist-th">Progress</th>
                  <th class="hist-th">Started</th>
                  <th class="hist-th">Eligible At</th>
                  <th class="hist-th">Last Completed</th>
                  <th class="hist-th">Last Status</th>
                  <th class="hist-th">Actions</th>
                </tr>
              </thead>
              <tbody>${statesHtml}</tbody>
            </table>
          </div>
        </div>

        <!-- Run History Log -->
        <div class="panel hist-section">
          <h3 class="hist-section-title">Run History &mdash; ${escapeHtml(historyRangeLabel())}</h3>
          <div class="hist-table-wrap">
            <table class="hist-table">
              <thead>
                <tr>
                  <th class="hist-th">Protocol</th>
                  <th class="hist-th">Status</th>
                  ${showDateColumn ? '<th class="hist-th">Date</th>' : ''}
                  <th class="hist-th">Start</th>
                  <th class="hist-th">End</th>
                  <th class="hist-th">Duration</th>
                  <th class="hist-th">Failure Reason</th>
                  <th class="hist-th">Detail</th>
                </tr>
              </thead>
              <tbody>${logRows}</tbody>
            </table>
          </div>
          ${filtered.length > (h.pageSize || 20) ? `
          <div class="hist-pager">
            <button class="btn btn-sm btn-outline" data-action="hist-prev-page" ${pageInfo.hasPrev ? '' : 'disabled'} title="Previous page">&lsaquo; Prev</button>
            <span class="hist-pager-label">Page ${pageInfo.page} of ${pageInfo.totalPages}</span>
            <button class="btn btn-sm btn-outline" data-action="hist-next-page" ${pageInfo.hasNext ? '' : 'disabled'} title="Next page">Next &rsaquo;</button>
          </div>` : ''}
        </div>
      </div>
    `;
  }

  function handleHistoryAction(target) {
    const action = target.dataset?.action;
    if (!action) return false;

    if (action === 'hist-mode') {
      state.history.mode = target.value;
      if (state.history.mode !== 'single') disconnectSSE();
      state.history.page = 1;
      // Mode switch should reset local filters to avoid confusing hidden rows.
      state.history.statusFilter = '';
      state.history.protocolFilter = '';
      if (state.history.mode === 'single' && !state.history.date) {
        state.history.date = new Date().toISOString().slice(0, 10);
      }
      if (state.history.mode === 'range') {
        if (!state.history.dateFrom) state.history.dateFrom = state.history.date || new Date().toISOString().slice(0, 10);
        if (!state.history.dateTo) state.history.dateTo = state.history.date || new Date().toISOString().slice(0, 10);
      }
      fetchHistory();
      return true;
    }
    if (action === 'hist-date') {
      state.history.date = target.value;
      state.history.page = 1;
      fetchHistory();
      return true;
    }
    if (action === 'hist-date-from') {
      state.history.dateFrom = target.value;
      state.history.page = 1;
      if (state.history.dateFrom && state.history.dateTo) fetchHistory();
      return true;
    }
    if (action === 'hist-date-to') {
      state.history.dateTo = target.value;
      state.history.page = 1;
      if (state.history.dateFrom && state.history.dateTo) fetchHistory();
      return true;
    }
    if (action === 'hist-status') {
      state.history.statusFilter = target.value;
      state.history.page = 1;
      renderProtocolHistory();
      return true;
    }
    if (action === 'hist-protocol') {
      state.history.protocolFilter = target.value;
      state.history.page = 1;
      renderProtocolHistory();
      return true;
    }
    if (action === 'hist-refresh') {
      state.history.page = 1;
      fetchHistory();
      return true;
    }
    if (action === 'hist-clear-filters') {
      state.history.statusFilter = '';
      state.history.protocolFilter = '';
      state.history.page = 1;
      fetchHistory();
      return true;
    }
    if (action === 'hist-prev-page') {
      state.history.page = Math.max(1, (state.history.page || 1) - 1);
      renderProtocolHistory();
      return true;
    }
    if (action === 'hist-next-page') {
      state.history.page = (state.history.page || 1) + 1;
      renderProtocolHistory();
      return true;
    }
    if (action === 'hist-set-idle') {
      const protocol = target.dataset?.protocol;
      if (protocol) setProtocolState(protocol, 'idle');
      return true;
    }
    return false;
  }

  async function setProtocolState(protocol, newState) {
    try {
      await api(`/api/protocol-state/${encodeURIComponent(protocol)}`, {
        method: 'PATCH',
        body: JSON.stringify({ state: newState }),
      });
      setFlash('ok', `${shortProtoName(protocol)} set to ${newState}`);
      // Refresh states immediately
      const stateRes = await api('/api/protocol-state');
      state.history.states = stateRes.states || [];
      renderProtocolHistory();
    } catch (err) {
      setFlash('error', `Failed to update: ${err.message}`);
    }
  }

  function attachEvents() {
    document.addEventListener('click', (e) => {
      const navBtn = e.target.closest('.nav-item');
      if (navBtn) {
        const route = navBtn.dataset.route;
        setRoute(route);
        if (route === 'protocol-history') fetchHistory();
        return;
      }
      const sidebarToggle = e.target.closest('#sidebarToggle');
      if (sidebarToggle) {
        state.ui.sidebarCollapsed = !state.ui.sidebarCollapsed;
        renderSidebarMode();
        return;
      }
      // History button actions (exclude inputs/selects — those use change event)
      const histBtn = e.target.closest('[data-action^="hist-"]');
      if (histBtn && histBtn.tagName !== 'INPUT' && histBtn.tagName !== 'SELECT') {
        handleHistoryAction(histBtn);
        return;
      }
      handleClick(e);
    });
    document.addEventListener('keydown', (e) => {
      const target = e.target.closest('[data-action="section-toggle"]');
      if (!target) return;
      if (e.key !== 'Enter' && e.key !== ' ') return;
      e.preventDefault();
      target.click();
    });
    document.addEventListener('change', (e) => {
      if (handleHistoryAction(e.target)) return;
      handleChange(e);
    });
    document.addEventListener('input', handleInput);
    window.addEventListener('beforeunload', (e) => {
      if (!state.dirty) return;
      e.preventDefault();
      e.returnValue = '';
    });
  }

  attachEvents();
  renderNav();
  renderSidebarMode();
  renderFlash();
  bootstrap();
})();
