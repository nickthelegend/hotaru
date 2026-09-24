"""sw_fresh.py -- restart the SolidWorks session this project drives and
clear the startup "Display scaling optimization" prompt that fires on
every launch on a mixed-resolution desk (it blocks the API until closed)."""
import ctypes
import subprocess
import sys
import time

sys.path.insert(0, __import__("os").path.dirname(__import__("os").path.abspath(__file__)))

u = ctypes.windll.user32


def windows(pid):
    out = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)
    def cb(h, _l):
        p = ctypes.c_ulong()
        u.GetWindowThreadProcessId(h, ctypes.byref(p))
        if p.value == pid and u.IsWindowVisible(h):
            t = ctypes.create_unicode_buffer(256)
            c = ctypes.create_unicode_buffer(64)
            u.GetWindowTextW(h, t, 256)
            u.GetClassNameW(h, c, 64)
            out.append((h, t.value, c.value))
        return True
    u.EnumWindows(cb, 0)
    return out


PS_KEEP = r"""
Add-Type -AssemblyName UIAutomationClient, UIAutomationTypes
Add-Type -Name U3 -Namespace W -MemberDefinition '[DllImport("user32.dll")] public static extern IntPtr PostMessage(IntPtr h, uint m, IntPtr w, IntPtr l); [DllImport("user32.dll")] public static extern bool ScreenToClient(IntPtr h, ref System.Drawing.Point p);' -ReferencedAssemblies System.Drawing
$root = [System.Windows.Automation.AutomationElement]::FromHandle([IntPtr]HWND)
$c = New-Object System.Windows.Automation.PropertyCondition([System.Windows.Automation.AutomationElement]::NameProperty, "Keep the current layout")
$e = $root.FindFirst([System.Windows.Automation.TreeScope]::Descendants, $c)
$r = $e.Current.BoundingRectangle; $h = [IntPtr]$e.Current.NativeWindowHandle
$pt = New-Object System.Drawing.Point([int]($r.X + 40), [int]($r.Y + $r.Height / 2))
[W.U3]::ScreenToClient($h, [ref]$pt) | Out-Null
$lp = [IntPtr](($pt.Y -shl 16) -bor ($pt.X -band 0xFFFF))
[W.U3]::PostMessage($h, 0x0201, [IntPtr]1, $lp) | Out-Null
Start-Sleep -Milliseconds 80
[W.U3]::PostMessage($h, 0x0202, [IntPtr]0, $lp) | Out-Null
"""


def keep_layout(hwnd):
    """Pick "Keep the current layout" -- the option that changes nothing.
    The prompt is a task dialog whose options are UI-Automation panes with
    no Invoke pattern, and it ignores WM_CLOSE, so the pane gets a click
    message posted straight to its own window (the mouse never moves)."""
    subprocess.run(["powershell", "-NoProfile", "-Command",
                    PS_KEEP.replace("HWND", str(hwnd))], capture_output=True)


def fresh(kill_pid=None):
    if kill_pid:
        subprocess.run(["taskkill", "/PID", str(kill_pid), "/F"], capture_output=True)
        time.sleep(3)
    import swlib as L
    sw = L.SW()
    h = sw.hwnd()
    pid = ctypes.c_ulong()
    u.GetWindowThreadProcessId(h, ctypes.byref(pid))
    for _ in range(40):                       # the prompt shows a few s in
        hit = [wh for wh, title, cls in windows(pid.value)
               if cls == "#32770" and title.startswith("SOLIDWORKS")]
        if hit:
            keep_layout(hit[0])
            print("startup prompt: kept the current layout")
            break
        time.sleep(0.5)
    sw.close_all()
    print("SolidWorks ready, pid", pid.value)
    return pid.value


if __name__ == "__main__":
    fresh(int(sys.argv[1]) if len(sys.argv) > 1 else None)
