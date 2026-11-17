"""Starting ScenarioRunner as a subprocess with the right paths."""

import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

from carsim.paths import CARLA_ROOT, REPO_ROOT, SCENARIO_RUNNER_ROOT


# The metrics block (carsim.blocks.metrics) writes its JSON to the file this variable names, when it is set
METRICS_FILE_ENV = "CARSIM_METRICS_FILE"


def scenario_runner_env(metrics_file=None):
    # Absolute paths: the subprocess runs in ScenarioRunner's folder, where a relative path would point elsewhere
    scenario_runner_root = os.path.abspath(SCENARIO_RUNNER_ROOT)
    env = os.environ.copy()
    env["SCENARIO_RUNNER_ROOT"] = scenario_runner_root
    # ScenarioRunner prints box-drawing characters (cp1252 can't encode them when piped)
    # and would otherwise block-buffer its output
    env["PYTHONUTF8"] = "1"
    env["PYTHONUNBUFFERED"] = "1"
    paths = [os.path.abspath(REPO_ROOT / "src"), os.path.abspath(CARLA_ROOT / "PythonAPI" / "carla"),
             scenario_runner_root]
    if env.get("PYTHONPATH"):
        paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(paths)
    env.pop(METRICS_FILE_ENV, None)  # only the file of this run, never one inherited from the shell
    if metrics_file is not None:
        env[METRICS_FILE_ENV] = os.path.abspath(metrics_file)
    return env


def run_scenario_runner(args, log_file=None, on_line=None, on_interrupt=None, metrics_file=None):
    """
    Run scenario_runner.py with `args`, streaming its output to the console
    (and to `log_file` if given). `on_line`, if given, is called with every
    output line (including its newline) after it has been written, so the
    caller can react to a milestone such as "ScenarioManager: Running scenario".
    An exception from `on_line` is printed and does not stop the run.
    `on_interrupt`, if given, is called without arguments when Ctrl+C ends the
    run (it is handled here, so the caller would not otherwise see it). After
    the first Ctrl+C the output is still read until ScenarioRunner exits (it
    prints its results while cleaning up); a second one only prints a note, a
    third abandons it (terminate, kill after 3 s). `metrics_file`, if given, is the path the metrics block writes its
    JSON to (environment variable CARSIM_METRICS_FILE). Returns the exit code.
    """
    scenario_runner_root = os.path.abspath(SCENARIO_RUNNER_ROOT)
    script = Path(scenario_runner_root) / "scenario_runner.py"
    if not script.is_file():
        raise FileNotFoundError(f"{script} not found. Run the setup first (scripts/setup_env_linux.sh, or scripts/setup_env.py).")

    # ScenarioRunner's result table uses box-drawing characters, which a cp1252 console or pipe
    # cannot encode; print them as '?' rather than crash after the run
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(errors="replace")
    cmd = [sys.executable, str(script)] + [str(a) for a in args]
    print("> " + " ".join(cmd))
    proc = subprocess.Popen(cmd, cwd=scenario_runner_root, env=scenario_runner_env(metrics_file),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            encoding="utf-8", errors="replace", bufsize=1)
    log = open(log_file, "w", encoding="utf-8") if log_file else None
    interrupts = 0
    try:
        # Ctrl+C reaches ScenarioRunner too; it stops, removes its actors, resets sync mode and prints its results
        # (a few KB: more than a pipe holds, so it blocks in print unless we keep reading). So after a Ctrl+C this
        # keeps reading and forwarding until EOF. terminate() would kill ScenarioRunner before it cleaned up.
        while True:
            try:
                for line in proc.stdout:
                    sys.stdout.write(line)
                    if log:
                        log.write(line)
                    if on_line is not None:
                        try:
                            on_line(line)
                        except Exception as e:  # a failing hook must not kill the output loop
                            print(f"on_line hook failed: {e}")
                proc.wait()
                break
            except KeyboardInterrupt:
                interrupts += 1
                if interrupts == 1:
                    if on_interrupt is not None:
                        try:
                            on_interrupt()
                        except Exception as e:
                            print(f"on_interrupt hook failed: {e}")
                    print("\nWaiting for ScenarioRunner to clean up...")
                elif interrupts == 2:
                    print("\nStill cleaning up, press Ctrl-C again to abandon.")
                else:
                    print("\nAbandoning ScenarioRunner: terminating it (its actors may be left in the world).")
                    try:
                        stop_process(proc, wait=3.0)
                    except KeyboardInterrupt:
                        proc.kill()
                        proc.wait()
                    break
    finally:
        if log:
            log.close()
    return proc.returncode


# The first ScenarioRunner line after the scenario, and so the ego, exists: "Preparing scenario" is printed before
# the ego is spawned and the scenario (with all its actors) is built, this one right after both
SCENARIO_RUNNING_LINE = "ScenarioManager: Running scenario"

# ScenarioRunner prints this only when the ego still exists at its own cleanup. At a normal end
# CarlaDataProvider.cleanup has already destroyed the ego, and ScenarioRunner then skips the print; so this line is
# not a reliable end marker by itself (see SCENARIO_ENDING_LINE). Either way the driving window exits by itself
# right after the ego is gone, which is not the user closing it.
EGO_DESTROYED_LINE = "Destroying ego vehicle"

# ComposedScenario.terminate() prints this at its start, before CarlaDataProvider.cleanup destroys the ego, at every
# end of a run (normal end, stop file, Ctrl+C, error): the definitive "the run is ending" marker
SCENARIO_ENDING_LINE = "ComposedScenario: scenario ending"


def manual_control_script():
    """ScenarioRunner's manual_control.py, the pygame window in which the user drives the ego."""
    return Path(os.path.abspath(SCENARIO_RUNNER_ROOT)) / "manual_control.py"


def manual_control_arguments(host="127.0.0.1", port=2000):
    """
    The command-line arguments of manual_control.py for a server: --keep_ego_vehicle always (the ego belongs to
    ScenarioRunner; without the flag closing the window, by Esc, its X or Ctrl+C, destroys the ego, and
    ScenarioRunner could not end the run cleanly), --host and --port only when not the defaults.
    """
    args = ["--keep_ego_vehicle"]
    if host != "127.0.0.1":
        args += ["--host", host]
    if port != 2000:
        args += ["--port", str(port)]
    return args


def start_window(cmd, cwd, log_file=None):
    """
    Start a pygame window program (`cmd`) as a child process and return the Popen. On Windows it gets its own console,
    so its output does not mix into the scenario output and the window opens and takes focus like a normally started
    program. Elsewhere (Linux, DISPLAY must name the X display) it gets its own session, so a Ctrl-C in this terminal
    reaches ScenarioRunner but not the window (as with its own console), and its output goes to `log_file` when given.
    """
    if os.name == "nt":
        return subprocess.Popen(cmd, cwd=cwd, env=scenario_runner_env(), creationflags=subprocess.CREATE_NEW_CONSOLE)
    if not os.environ.get("DISPLAY"):
        print("WARNING: DISPLAY is not set, the window cannot open (e.g. docker exec -e DISPLAY=:0 ...).")
    out = open(log_file, "a", encoding="utf-8") if log_file else None
    try:
        return subprocess.Popen(cmd, cwd=cwd, env=scenario_runner_env(), start_new_session=True,
                                stdout=out, stderr=subprocess.STDOUT if out else None)
    finally:
        if out:
            out.close()  # the child keeps its own copy


def manual_control_wrapper():
    """apps/manual_control_tm.py: manual_control.py with its set_autopilot calls on our Traffic Manager port."""
    return REPO_ROOT / "apps" / "manual_control_tm.py"


def start_manual_control(host="127.0.0.1", port=2000, log_file=None, tm_port=8100):
    """
    Start manual_control.py (the driving window) as a child process and return the Popen (see start_window): this
    interpreter, ScenarioRunner's folder and environment, through manual_control_wrapper() so that its
    set_autopilot(False) uses Traffic Manager `tm_port` (CARLA would make one on 8000).
    """
    script = manual_control_script()
    if not script.is_file():
        raise FileNotFoundError(f"{script} not found. Run the setup first (scripts/setup_env_linux.sh, or scripts/setup_env.py).")
    cmd = [sys.executable, str(manual_control_wrapper()), "--tm-port", str(tm_port)] + manual_control_arguments(host, port)
    return start_window(cmd, os.path.dirname(str(script)), log_file)


def stop_process(proc, wait=2.0, grace=0.0):
    """
    Stop a child process if it is still running: first give it up to `grace` seconds to exit by itself, then
    terminate it, and kill it when it has not exited within `wait` more seconds. Returns True when it had to be
    stopped, False when it had already exited, or exited within `grace` (or `proc` is None).
    """
    if proc is None or proc.poll() is not None:
        return False
    if grace > 0:
        try:
            proc.wait(grace)
            return False
        except subprocess.TimeoutExpired:
            pass
    proc.terminate()
    try:
        proc.wait(wait)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    return True


def chase_view_script():
    """apps/chase_view.py, the pygame window that follows the ego (the viewer for autopilot runs)."""
    return REPO_ROOT / "apps" / "chase_view.py"


def chase_view_command_args(host="127.0.0.1", port=2000, control_dir=None, spectator=False):
    """The command-line arguments of chase_view.py for a server: --host and --port only when not the defaults, --control with a `control_dir`, --spectator."""
    args = []
    if control_dir is not None:
        args += ["--control", str(control_dir)]
    if spectator:
        args.append("--spectator")
    if host != "127.0.0.1":
        args += ["--host", host]
    if port != 2000:
        args += ["--port", str(port)]
    return args


def start_chase_view(host="127.0.0.1", port=2000, control_dir=None, spectator=False, log_file=None):
    """
    Start chase_view.py (the chase-view window) as a child process and return the Popen.

    Like start_manual_control (see start_window): this interpreter, ScenarioRunner's environment, its own console on
    Windows, its own session and `log_file` elsewhere. The viewer waits for the hero by itself and closes when the hero
    is gone; with a `control_dir` (see the control files below) it stays open instead. `spectator`: it moves the
    server's spectator behind the ego instead of attaching a camera sensor (see apps/chase_view.py).
    """
    script = chase_view_script()
    if not script.is_file():
        raise FileNotFoundError(f"{script} not found.")
    cmd = [sys.executable, str(script)] + chase_view_command_args(host, port, control_dir, spectator)
    return start_window(cmd, str(REPO_ROOT), log_file)


# The chase view as the control panel of an autopilot run: small files in a per-session folder (no sockets). The window
# writes the requests; run_scenario.py reads and removes them and writes the result of every run for the window.
RESTART_REQUEST = "restart.request"  # written by the window (R, the Restart button)
QUIT_REQUEST = "quit.request"  # written by the window (Esc, the X, the Quit button), and by run_scenario at its end
RESULT_FILE = "result.json"  # written by run_scenario after each run: {"run", "result", "summary", "run_dir"}

RESULT_LINE = re.compile(r"Results of Scenario: .* ---- (\w+) =")  # ScenarioRunner's result table header


def parse_result_line(line):
    """SUCCESS / FAILURE / ... from ScenarioRunner's "Results of Scenario: <name> ---- <result> =======" line, else None."""
    match = RESULT_LINE.search(line)
    return match.group(1) if match else None


def write_request(control_dir, name):
    Path(control_dir, name).touch()


def clear_request(control_dir, name):
    Path(control_dir, name).unlink(missing_ok=True)


def pending_request(control_dir):
    """'quit' (wins), 'restart', or None: what the window asked for. Does not remove anything."""
    for name, request in ((QUIT_REQUEST, "quit"), (RESTART_REQUEST, "restart")):
        if Path(control_dir, name).exists():
            return request
    return None


def write_result(control_dir, run, result, summary, run_dir):
    """The result of run number `run` for the window: `result` is ScenarioRunner's (None when unknown), `summary` the metrics line or None."""
    path = Path(control_dir, RESULT_FILE)
    tmp = path.with_suffix(".tmp")  # replaced in one step: the window must never read half a file
    tmp.write_text(json.dumps({"run": run, "result": result, "summary": summary, "run_dir": str(run_dir)}), encoding="utf-8")
    os.replace(tmp, path)


def read_result(control_dir):
    """The dict write_result wrote, or None (no run has ended yet, or the file is not readable)."""
    try:
        data = json.loads(Path(control_dir, RESULT_FILE).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def clear_result(control_dir):
    Path(control_dir, RESULT_FILE).unlink(missing_ok=True)


def next_step(request, interrupted, viewer_alive):
    """
    What run_scenario does when a run is over: 'end' (Ctrl-C, or no window to ask: a single run as without a window),
    'quit' or 'restart' (the window asked), 'wait' (the window stays open and asks next).
    """
    if interrupted:
        return "end"
    if request in ("quit", "restart"):
        return request
    return "wait" if viewer_alive else "end"


def wait_for_request(control_dir, viewer_alive, poll=0.2):
    """Block until the window asks for something: 'restart' or 'quit'. 'quit' too when the window is gone (`viewer_alive()` false)."""
    while True:
        request = pending_request(control_dir)
        if request is not None:
            return request
        if not viewer_alive():
            return "quit"
        time.sleep(poll)


def watch_requests(control_dir, on_request, done, poll=0.2):
    """Run in a thread during a run: call `on_request(request)` once when the window asks for something, until the Event `done` is set."""
    while not done.wait(poll):
        request = pending_request(control_dir)
        if request is not None:
            on_request(request)
            return
