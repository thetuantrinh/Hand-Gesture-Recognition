# Qt Designer source

`UI.ui` is the Qt Designer document the main window layout originated from.

The generated Python was produced with:

```bash
pyuic5 -x UI.ui -o main_window_ui.py
```

**The generated output is no longer used directly.** It was hand-edited (asset
loading, the log console, the gripper radio buttons) and has since been split
into the panel mixins under [`../layout/`](../layout), which is the
maintained source. `UI.ui` is kept for reference when reasoning about the
original geometry — regenerating from it would discard the hand-written
additions.
