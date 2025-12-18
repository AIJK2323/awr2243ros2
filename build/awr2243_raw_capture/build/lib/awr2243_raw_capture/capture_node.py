import os
import time
import shlex
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def run_cmd(cmd, cwd=None, logger=None):
    if isinstance(cmd, str):
        cmd_list = shlex.split(cmd)
    else:
        cmd_list = cmd
    if logger:
        logger.info(f"Executing: {cmd_list} (cwd={cwd})")
    p = subprocess.Popen(cmd_list, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    out, err = p.communicate()
    if logger:
        if out.strip():
            logger.info(out.strip())
        if err.strip():
            logger.warn(err.strip())
    return p.returncode, out, err

class Awr2243RawCapture(Node):
    def __init__(self):
        super().__init__('awr2243_raw_capture')

        # Parameters
        self.declare_parameter('work_dir', '$HOME/ros2_ws/src/awr2243_raw_capture/work_dir/')  # directory containing CLI binaries and cf.json
        self.declare_parameter('control_exe', './DCA1000EVM_CLI_Control')
        self.declare_parameter('record_exe', './DCA1000EVM_CLI_Record')  # optional: ./DCA1000EVM_CLI_Record
        self.declare_parameter('cfg_json', 'cf.json')
        self.declare_parameter('output_dir', 'data')
        self.declare_parameter('file_prefix', 'raw_data')
        self.declare_parameter('duration_sec', 5.0)
        self.declare_parameter('auto_fpga_config', True)
        self.declare_parameter('auto_record_config', True)

        self.pub = self.create_publisher(String, 'awr2243/raw_file', 10)

        self._timer = self.create_timer(0.5, self._once)
        self._ran = False

    def _once(self):
        if self._ran:
            return
        self._ran = True

        work_dir = self.get_parameter('work_dir').get_parameter_value().string_value
        control_exe = self.get_parameter('control_exe').get_parameter_value().string_value
        record_exe  = self.get_parameter('record_exe').get_parameter_value().string_value
        cfg_json    = self.get_parameter('cfg_json').get_parameter_value().string_value
        output_dir  = self.get_parameter('output_dir').get_parameter_value().string_value
        file_prefix = self.get_parameter('file_prefix').get_parameter_value().string_value
        dur         = self.get_parameter('duration_sec').get_parameter_value().double_value
        auto_fpga   = self.get_parameter('auto_fpga_config').get_parameter_value().bool_value
        auto_record = self.get_parameter('auto_record_config').get_parameter_value().bool_value

        work = Path(work_dir)
        cfg = work / cfg_json
        print(cfg)

        if not cfg.exists():
            self.get_logger().error(f"Config json not found: {cfg}")
            return

        # Ensure output directory exists (DCA1000 CLI uses 'saveFileName' prefix in cfg)
        Path(output_dir).mkdir(parents=True, exist_ok=True)

        # Optionally patch the cfg.json's file prefix at runtime
        try:
            import json
            data = json.loads(cfg.read_text())
            d = data.get('DCA1000Config', {})
            cap = d.get('captureConfig', {})
            save_path = str(Path(output_dir) / file_prefix)
            if 'saveFileName' in cap:
                cap['saveFileName'] = save_path
            elif 'fileBasePath' in cap and 'filePrefix' in cap:
                cap['fileBasePath'] = str(Path(output_dir))
                cap['filePrefix'] = file_prefix
            d['captureConfig'] = cap
            data['DCA1000Config'] = d
            tmp_cfg = work / ('._tmp_cf.json')
            tmp_cfg.write_text(json.dumps(data, indent=2))
            cfg_to_use = tmp_cfg.name
        except Exception as e:
            self.get_logger().warn(f"Could not patch cfg.json for output name, using original. ({e})")
            cfg_to_use = cfg.name

        # Sequence (mirrors the repo's usage):
        if auto_fpga:
            run_cmd([control_exe, 'fpga', cfg_to_use], cwd=str(work), logger=self.get_logger())
        if auto_record:
            run_cmd([control_exe, 'record', cfg_to_use], cwd=str(work), logger=self.get_logger())

        # Optional: start dedicated recorder if provided
        recorder_proc = None
        if record_exe:
            try:
                recorder_proc = subprocess.Popen([record_exe, cfg_to_use], cwd=str(work))
                self.get_logger().info(f"Spawned DCA1000EVM_CLI_Record (pid={recorder_proc.pid})")
                time.sleep(0.2)
            except Exception as e:
                self.get_logger().warn(f"Failed to start {record_exe}: {e}")

        rc, _, _ = run_cmd([control_exe, 'start_record', cfg_to_use], cwd=str(work), logger=self.get_logger())
        if rc != 0:
            self.get_logger().error("start_record failed")
            return

        self.get_logger().info(f"Capturing for {dur:.2f} s ...")
        time.sleep(max(0.0, dur))

        run_cmd([control_exe, 'stop_record', cfg_to_use], cwd=str(work), logger=self.get_logger())

        if recorder_proc is not None:
            try:
                recorder_proc.terminate()
            except Exception:
                pass

        out = String()
        out.data = str(Path(output_dir) / file_prefix)
        self.pub.publish(out)
        self.get_logger().info(f"Published output prefix: {out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Awr2243RawCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
