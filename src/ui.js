const namespace = 'http://www.w3.org/2000/svg';

const icons = {
  mechanism: '<path d="M4 18 12 5l8 13M4 18h16"/><circle cx="4" cy="18" r="2" fill="#28664f"/><circle cx="12" cy="5" r="2" fill="#28664f"/><circle cx="20" cy="18" r="2" fill="#28664f"/>',
  menu: '<path d="M4 5h16M4 12h16M4 19h16"/>',
  close: '<path d="m6 6 12 12M18 6 6 18"/>',
  new: '<path d="M14 2H6a2 2 0 0 0-2 2v16a2 2 0 0 0 2 2h12a2 2 0 0 0 2-2V8l-6-6v6h6M9 14h6m-3-3v6"/>',
  open: '<path d="M3 18V5h6l3 3h8v3M3 18l4-7h15l-4 7z"/>',
  export: '<path d="M4 15v5h16v-5M12 4v11m-4-4 4 4 4-4"/>',
  help: '<circle cx="12" cy="12" r="9"/><path d="M9.5 9a2.5 2.5 0 1 1 4.5 1.5c-1 1-2 1-2 3m0 3h.01"/>',
  undo: '<path d="m7 5-5 5 5 5m-5-5h13a4 4 0 0 1 0 8h-3"/>',
  redo: '<path d="m17 5 5 5-5 5m5-5H9a4 4 0 0 0 0 8h3"/>',
  save: '<path d="M5 3h11l5 5v11a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2zM8 3v5h8V3M7 21v-8h10v8"/>',
  select: '<path d="M4 4l16 7-7 2-2 7z"/>',
  pan: '<path d="M7 11V6a1.5 1.5 0 0 1 3 0v4-6a1.5 1.5 0 0 1 3 0v6-5a1.5 1.5 0 0 1 3 0v6-3a1.5 1.5 0 0 1 3 0v7a6 6 0 0 1-6 6h-1c-2 0-3.4-.8-4.7-2.5L4 14a1.6 1.6 0 0 1 2.5-2l1.2 1.2"/>',
  tree: '<path d="M6 4h6M6 4v15m0-6h6m-6 6h10"/><path d="M12 3h4v3h-4zm0 8h4v3h-4zm4 6h4v4h-4z"/>',
  eye: '<path d="M2 12s4-6 10-6 10 6 10 6-4 6-10 6-10-6-10-6z"/><circle cx="12" cy="12" r="2.5"/>',
  rod: '<path d="M6 18 18 6"/><circle cx="4.5" cy="19.5" r="2"/><circle cx="19.5" cy="4.5" r="2"/>',
  body: '<path d="m4 16 12-12 4 4L8 20z"/><circle cx="7" cy="17" r="1"/><circle cx="17" cy="7" r="1"/>',
  'pin-joint': '<circle cx="12" cy="12" r="4"/><path d="M12 4v2m0 12v2M4 12h2m12 0h2"/>',
  fixed: '<path d="M7 6h10v10H7zM4 17h16m-12 0-3 4m7-4-3 4m7-4-3 4m7-4-3 4"/>',
  rotation: '<circle cx="12" cy="7" r="2"/><path d="m11 9-5 8h12l-5-8M4 17h16m-12 0-3 4m7-4-3 4m7-4-3 4m7-4-3 4"/>',
  ruler: '<path d="M2 14 14 2l8 8-12 12zM6 10l3 3m1-7 3 3m1-7 3 3"/>',
  coincident: '<path d="M9 7H7a4 4 0 0 0 0 8h2m6 0h2a4 4 0 0 0 0-8h-2M8 11h8"/>',
  horizontal: '<path d="M3 12h18M3 8v8m18-8v8"/>',
  vertical: '<path d="M12 3v18M8 3h8m-8 18h8"/>',
  equal: '<path d="M4 9h16M4 15h16"/>',
  parallel: '<path d="M5 19 15 5M9 19 19 5"/>',
  force: '<path d="M12 3v17m-5-5 5 5 5-5"/>',
  actuator: '<path d="M5 19 8 16m4 0 5-5-4-4-5 5zm3-7 4-4"/><circle cx="4" cy="20" r="1.5"/><circle cx="20" cy="4" r="1.5"/>',
  spring: '<path d="M3 21l4-4 3 1-2-5 5 2-2-5 5 2-2-5 3 1 4-4"/>',
  motor: '<circle cx="12" cy="12" r="8"/><path d="M8.5 15V9l3.5 4 3.5-4v6M12 2v2m0 16v2"/>',
  play: '<path d="m7 4 14 8-14 8z"/>',
  pause: '<path d="M8 5v14M16 5v14" stroke-width="3"/>',
  step: '<path d="m5 5 11 7-11 7zM19 5v14"/>',
  reset: '<path d="M3 12a9 9 0 1 1 2.6 6.4M3 5v7h7"/>',
  settings: '<path d="M4 6h16M4 12h16M4 18h16M9 3v6m6 0v6m-6 0v6"/>',
  trace: '<path d="M4 19 9 10l5 4 6-9" stroke-dasharray="2 3"/><circle cx="4" cy="19" r="1.5"/><circle cx="20" cy="5" r="1.5"/>',
  trash: '<path d="M4 6h16M9 6V3h6v3M6 6l1 15h10l1-15M10 10v7m4-7v7"/>',
  results: '<path d="M4 19V5m0 14h16m-13-3 4-5 4 2 4-6M16 7h3v3"/>',
  pin: '<path d="m15 3 6 6-3 1-3 4v4l-3-3-6-6h4l4-3zM9 15l-6 6"/>',
  'pin-off': '<path d="m16 4 4 4-3 2-1 3M5 4l15 15M8 8 6 9l8 8v-2m-5-2-6 6"/>',
  down: '<path d="m6 9 6 6 6-6"/>',
  'zoom-out': '<circle cx="10.5" cy="10.5" r="6"/><path d="m15 15 5 5M8 10.5h5"/>',
  'zoom-in': '<circle cx="10.5" cy="10.5" r="6"/><path d="m15 15 5 5M8 10.5h5m-2.5-2.5v5"/>',
  fit: '<path d="M9 4H4v5m11-5h5v5M4 15v5h5m11-5v5h-5"/>',
};

function setIcon(element, name) {
  element.querySelector(':scope > svg.icon')?.remove();
  const svg = document.createElementNS(namespace, 'svg');
  svg.classList.add('icon');
  svg.setAttribute('aria-hidden', 'true');
  svg.setAttribute('viewBox', '0 0 24 24');
  svg.setAttribute('fill', 'none');
  svg.setAttribute('stroke', 'currentColor');
  svg.setAttribute('stroke-width', '1.7');
  svg.setAttribute('stroke-linecap', 'round');
  svg.setAttribute('stroke-linejoin', 'round');
  svg.innerHTML = icons[name];
  element.prepend(svg);
}

function loadRibbonPreference() {
  try {
    return localStorage.getItem('rod-mechanism-studio.ribbon-pinned') !== 'false';
  } catch {
    return true;
  }
}

export function setupStudio() {
  const studio = document.querySelector('.studio');
  const byId = id => document.getElementById(id);
  const workspace = byId('workspace');
  const menu = byId('main-menu');
  const menuButton = studio.querySelector('[data-shell="menu"]');
  const visibilityOptions = byId('visibility-options');
  const visibilityButton = studio.querySelector('[data-shell="visibility"]');
  const tree = byId('model-tree-panel');
  const treeButton = studio.querySelector('[data-panel="tree"]');
  const inspector = byId('inspector-panel');
  const dockButton = studio.querySelector('[data-action="dock-panel"]');
  const playButton = studio.querySelector('[data-action="play"]');
  const ribbon = byId('command-ribbon');
  const ribbonBody = byId('ribbon-body');
  const ribbonReveal = byId('ribbon-reveal');
  const ribbonPin = studio.querySelector('[data-shell="pin-ribbon"]');
  const help = byId('help-dialog');
  let pinned = loadRibbonPreference();
  let docked = false;

  studio.querySelectorAll('[data-icon]').forEach(element => setIcon(element, element.dataset.icon));

  function setExpanded(button, content, visible) {
    button.setAttribute('aria-expanded', String(visible));
    content.hidden = !visible;
  }

  function setTree(visible) {
    workspace.classList.toggle('with-tree', visible);
    setExpanded(treeButton, tree, visible);
  }

  function setRibbon(expanded = false) {
    ribbon.classList.toggle('auto-hide', !pinned);
    ribbonReveal.hidden = pinned;
    setExpanded(ribbonReveal, ribbonBody, pinned || expanded);
    ribbonPin.setAttribute('aria-pressed', String(pinned));
    ribbonPin.setAttribute('aria-label', pinned ? 'Unpin ribbon' : 'Pin ribbon');
    ribbonPin.title = pinned ? 'Unpin ribbon to collapse it between commands' : 'Keep the ribbon open';
    setIcon(ribbonPin, pinned ? 'pin' : 'pin-off');
  }

  function setPanel(title) {
    inspector.hidden = !title;
    if (title) byId('inspector-title').textContent = title;
    workspace.classList.toggle('inspector-docked', Boolean(title && docked));
  }

  function setMode(mode) {
    if (studio.dataset.modeCurrent === mode) return;
    studio.dataset.modeCurrent = mode;
    const name = {model:'Model tools', simulation:'Simulation tools', analysis:'Analysis tools'}[mode];
    ribbon.setAttribute('aria-label', name);
    ribbonReveal.querySelector('span').textContent = name;
    studio.querySelectorAll('[data-mode]').forEach(button => {
      button.setAttribute('aria-pressed', String(button.dataset.mode === mode));
    });
    studio.querySelectorAll('[data-show-mode]').forEach(element => {
      element.hidden = element.dataset.showMode !== mode;
    });
    studio.querySelectorAll('[data-hide-mode]').forEach(element => {
      element.hidden = element.dataset.hideMode === mode;
    });
    ribbon.querySelector('.ribbon-scroll').scrollLeft = 0;
  }

  function setTool(tool, constraint) {
    studio.querySelectorAll('[data-tool], [data-constraint]').forEach(button => {
      const active = button.dataset.tool ? button.dataset.tool === tool
        : tool === 'constraint' && button.dataset.constraint === constraint;
      button.setAttribute('aria-pressed', String(active));
    });
  }

  function setPlaying(playing) {
    byId('play-label').textContent = playing ? 'Pause' : 'Run';
    setIcon(playButton, playing ? 'pause' : 'play');
  }

  function setStatus(hint, model) {
    if (hint != null) byId('status-hint').textContent = hint;
    if (model != null) byId('model-status').textContent = model;
  }

  function setZoom(percent) {
    byId('zoom-level').textContent = String(percent).endsWith('%') ? percent : Math.round(percent) + '%';
  }

  studio.addEventListener('click', event => {
    const button = event.target.closest('button');
    if (!button) return;

    if (menu.contains(button)) setExpanded(menuButton, menu, false);

    switch (button.dataset.shell || button.dataset.action) {
      case 'menu':
        setExpanded(menuButton, menu, menu.hidden);
        break;
      case 'visibility':
        setExpanded(visibilityButton, visibilityOptions, visibilityOptions.hidden);
        break;
      case 'tree':
        setTree(tree.hidden);
        if (tree.hidden) treeButton.focus();
        break;
      case 'ribbon':
        setRibbon(ribbonBody.hidden);
        break;
      case 'pin-ribbon':
        pinned = !pinned;
        try {
          localStorage.setItem('rod-mechanism-studio.ribbon-pinned', String(pinned));
        } catch {
          // The ribbon can still be pinned for this page.
        }
        setRibbon();
        (pinned ? ribbonPin : ribbonReveal).focus();
        break;
      case 'dock-panel':
        docked = !docked;
        workspace.classList.toggle('inspector-docked', docked);
        dockButton.setAttribute('aria-pressed', String(docked));
        dockButton.setAttribute('aria-label', docked ? 'Float panel' : 'Dock panel');
        dockButton.title = docked ? 'Float panel' : 'Dock panel';
        setIcon(dockButton, docked ? 'pin-off' : 'pin');
        break;
      case 'close-panel':
        setPanel(null);
        break;
      case 'help':
        help.showModal();
        break;
    }

    if (!pinned && ribbonBody.contains(button) && button !== ribbonPin) {
      setRibbon();
      if (event.detail === 0) ribbonReveal.focus();
    }
  });

  document.addEventListener('pointerdown', event => {
    if (!menu.hidden && !menu.contains(event.target) && !menuButton.contains(event.target)) {
      setExpanded(menuButton, menu, false);
    }
    if (!visibilityOptions.hidden && !visibilityOptions.contains(event.target) && !visibilityButton.contains(event.target)) {
      setExpanded(visibilityButton, visibilityOptions, false);
    }
    if (!pinned && !ribbonBody.hidden && !ribbon.contains(event.target)) setRibbon();
  }, true);

  document.addEventListener('keydown', event => {
    if (help.open) {
      if (event.key === 'Escape') {
        event.preventDefault();
        help.close();
      }
      event.stopPropagation();
      return;
    }
    if (event.key !== 'Escape') return;
    if (!menu.hidden) {
      setExpanded(menuButton, menu, false);
      menuButton.focus();
    } else if (!visibilityOptions.hidden) {
      setExpanded(visibilityButton, visibilityOptions, false);
      visibilityButton.focus();
    } else if (!pinned && !ribbonBody.hidden) {
      setRibbon();
      ribbonReveal.focus();
    } else {
      return;
    }
    event.preventDefault();
    event.stopPropagation();
  }, true);

  help.addEventListener('close', () => menuButton.focus());
  setRibbon();
  setTool('select');

  return { setMode, setPanel, setPlaying, setStatus, setTool, setTree, setZoom };
}
