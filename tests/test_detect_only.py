#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# 端到端测试：solve_calibration.py 的 --detect-only 模式（标定向导第③步
# "拍完一轮判连通"依赖的那条检测路径）。
#
# 跟 test_round_connectivity.cpp 分工明确：那边测的是"角点数>0 => 算共视"
# 这层纯数学判据和并查集连通逻辑（不需要真图片、不需要 OpenCV）；这边测的
# 是"真图片喂进去，角点数到底对不对"——用 OpenCV 自己生成一张标准 ChArUco
# 板图像（cv2.aruco.CharucoBoard.generateImage，跟 make_charuco_detector 用
# 完全同一套板参数），加上一张全黑图、一张重度遮挡图，跑一次真实的
# --detect-only 子进程调用，断言三种情况下角点数分别符合预期：
#   cam0（完整正对的板子）—— 角点数明显 > 0
#   cam1（全黑，压根没有板子）—— 角点数 == 0
#   cam2（板子被挡住大半，只剩一角）—— 角点数 == 0（不足以插值出任何
#         ChArUco 角点，跟 CalibWizard.cpp 里"板子被挡住"这个真实场景对应）
#
# 这个脚本本身要求机器上装了 opencv-contrib-python 才能跑（跟被测的功能
# 本身要求一致）——没装的话打印 SKIP、退出码 0，不当成测试失败：没装
# OpenCV 是环境问题，不是这份代码逻辑的问题，这一点跟 C++ 侧
# detectBoardFolders() 检测器不可用时的优雅降级是同一个态度。
# ---------------------------------------------------------------------------
import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path


def skip(msg: str) -> None:
    print(f"[SKIP] {msg}")
    sys.exit(0)


def fail(msg: str) -> None:
    print(f"[FAIL] {msg}")
    sys.exit(1)


def main() -> None:
    try:
        import cv2
        import numpy as np
    except ImportError as e:
        skip(f"缺少依赖（{e}），需要 opencv-contrib-python + numpy 才能跑这个测试")
        return

    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "CharucoBoard"):
        skip("当前 cv2 没有 aruco.CharucoBoard（opencv-contrib 没装，或者版本太旧/太新，"
             "接口变了）——这是环境问题，不是代码逻辑问题，跳过不算失败")
        return

    script = Path(__file__).resolve().parent.parent / "tools" / "solve_calibration.py"
    if not script.exists():
        # 兼容直接在 tests/ 旁边摆一份 solve_calibration.py 跑的情况（比如
        # 这次对话里发给用户的独立文件，没有完整的 tools/ 目录结构）。
        alt = Path(__file__).resolve().parent.parent / "solve_calibration.py"
        script = alt if alt.exists() else script
    if not script.exists():
        skip(f"找不到 solve_calibration.py（找过 {script}）——检查测试脚本相对路径的假设是否成立")
        return

    board_params = {
        "dict": "DICT_5X5_100",
        "squaresX": 5,
        "squaresY": 7,
        "squareMM": 30.0,
        "markerMM": 22.0,
    }

    with tempfile.TemporaryDirectory() as tmp:
        sess = Path(tmp)
        (sess / "manifest.json").write_text(
            json.dumps({"board": board_params}), encoding="utf-8")
        round_dir = sess / "rounds" / "round_test"
        round_dir.mkdir(parents=True)

        # 用跟 make_charuco_detector 完全同一套参数构造板对象，生成一张标准
        # 正对图像——这样"检测器认不认得出"测的是真实代码路径，不是随便拍脑袋
        # 造一张图碰运气。
        aruco = cv2.aruco
        dic = aruco.getPredefinedDictionary(getattr(aruco, board_params["dict"]))
        board = aruco.CharucoBoard(
            (board_params["squaresX"], board_params["squaresY"]),
            board_params["squareMM"], board_params["markerMM"], dic)
        board.setLegacyPattern(True)

        full_img = board.generateImage((900, 1260))
        cv2.imwrite(str(round_dir / "cam0.png"), full_img)

        blank_img = np.zeros_like(full_img)
        cv2.imwrite(str(round_dir / "cam1.png"), blank_img)

        # 只留左上角一小块（不够插值出任何 ChArUco 角点，通常需要至少 2x2
        # 相邻方格），其余全部涂白——模拟"板子被挡住大半"。
        occluded_img = full_img.copy()
        h, w = occluded_img.shape[:2]
        keep_w, keep_h = int(w * 0.12), int(h * 0.12)
        occluded_img[:, keep_w:] = 255
        occluded_img[keep_h:, :keep_w] = 255
        cv2.imwrite(str(round_dir / "cam2.png"), occluded_img)

        proc = subprocess.run(
            [sys.executable, str(script), "--detect-only", str(sess), "round_test"],
            capture_output=True, text=True, timeout=30)

        if proc.returncode != 0:
            fail(f"--detect-only 子进程返回非0：{proc.returncode}\nstderr:\n{proc.stderr}")
            return

        # 跟 C++ 侧 detectBoardFolders() 同样的容错：只取输出里最后一段能
        # 解析成 JSON 的行，允许中间夹带其它诊断输出。
        result = None
        for line in reversed(proc.stdout.splitlines()):
            line = line.strip()
            if line.startswith("{"):
                try:
                    result = json.loads(line)
                    break
                except json.JSONDecodeError:
                    continue
        if result is None:
            fail(f"没能从 stdout 解析出 JSON，原始输出：\n{proc.stdout}\nstderr:\n{proc.stderr}")
            return

        checks = 0
        failures = 0

        def check(cond: bool, msg: str) -> None:
            nonlocal checks, failures
            checks += 1
            status = "PASS" if cond else "FAIL"
            print(f"[{status}] {msg}")
            if not cond:
                failures += 1

        cam0 = result.get("0", -1)
        cam1 = result.get("1", -1)
        cam2 = result.get("2", -1)

        check(cam0 >= 6, f"cam0（完整正对的板子）应检出足够角点（>=6），实际 {cam0}")
        check(cam1 == 0, f"cam1（全黑，无板子）应检出 0 个角点，实际 {cam1}")
        check(cam2 == 0, f"cam2（板子被挡住大半）应检出 0 个角点，实际 {cam2}")

        print(f"{checks} checks, {failures} failures")
        sys.exit(0 if failures == 0 else 1)


if __name__ == "__main__":
    main()
