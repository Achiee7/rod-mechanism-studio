(() => {
  const key = 'rod-mechanism-studio.appearance';
  const system = matchMedia('(prefers-color-scheme: dark)');
  let theme = 'blue';
  let mode = 'system';
  try {
    const saved = JSON.parse(localStorage.getItem(key));
    if (['blue', 'violet', 'teal', 'amber', 'slate'].includes(saved?.theme)) theme = saved.theme;
    if (['light', 'dark', 'system'].includes(saved?.mode)) mode = saved.mode;
  } catch { /* Appearance remains available when storage is restricted. */ }

  function apply() {
    const root = document.documentElement;
    root.dataset.theme = theme;
    root.dataset.mode = mode === 'system' ? (system.matches ? 'dark' : 'light') : mode;
    document.querySelector('meta[name="theme-color"]').content = root.dataset.mode === 'dark' ? '#121824' : '#ffffff';
  }
  apply();
  system.addEventListener('change', apply);
  document.addEventListener('DOMContentLoaded', () => {
    const themeSelect = document.getElementById('appearance-theme');
    const modeSelect = document.getElementById('appearance-mode');
    themeSelect.value = theme;
    modeSelect.value = mode;
    for (const select of [themeSelect, modeSelect]) {
      select.addEventListener('change', () => {
        theme = themeSelect.value;
        mode = modeSelect.value;
        apply();
        try { localStorage.setItem(key, JSON.stringify({ theme, mode })); }
        catch { /* Keep this session's selection. */ }
      });
    }
  });
})();
