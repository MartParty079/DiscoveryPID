from pathlib import Path
import json


def replace_once(path, old, new):
    p = Path(path)
    text = p.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{path}: expected one match, found {count}: {old!r}")
    p.write_text(text.replace(old, new, 1), encoding="utf-8")


replace_once(
    "src/main.cpp",
    "DiscoveryPID direct-position-pid-v3.3-300deg 2026-08-17",
    "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange 2026-08-18",
)
replace_once("src/main.cpp", "constexpr float MAX_POSITION_KP = 100.0f;", "constexpr float MAX_POSITION_KP = 1000.0f;")
replace_once("src/main.cpp", "constexpr float MAX_POSITION_KI = 100.0f;", "constexpr float MAX_POSITION_KI = 500.0f;")
replace_once("src/main.cpp", "constexpr float MAX_POSITION_KD = 20.0f;", "constexpr float MAX_POSITION_KD = 200.0f;")

replace_once(
    "tools/run_position_trial.py",
    'EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.3-300deg"',
    'EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange"',
)

replace_once(
    "jmp/DiscoveryPID_Position_V2.jsl",
    'studyName = "PositionStudy_DirectPID_v37_300DegWide";',
    'studyName = "PositionStudy_DirectPID_v38_300Deg10xRange";',
)
replace_once("jmp/DiscoveryPID_Position_V2.jsl", "kpHigh = 100.0;", "kpHigh = 1000.0;")
replace_once("jmp/DiscoveryPID_Position_V2.jsl", "kiHigh = 50.0;", "kiHigh = 500.0;")
replace_once("jmp/DiscoveryPID_Position_V2.jsl", "kdHigh = 20.0;", "kdHigh = 200.0;")
replace_once(
    "jmp/DiscoveryPID_Position_V2.jsl",
    'projectDirectory || "data/position_autotune_directpid_v37_300degwide";',
    'projectDirectory || "data/position_autotune_directpid_v38_300deg10xrange";',
)

old_seeds = '''seedSettings = {
    {2.0,   0.0,  0.20},
    {10.0,  0.0,  1.00},
    {25.0,  2.5,  3.00},
    {50.0,  5.0,  6.00},
    {75.0, 15.0, 10.00},
    {90.0, 30.0, 15.00},
    {100.0, 50.0, 20.00}
};'''
new_seeds = '''seedSettings = {
    // Keep a low-gain baseline and the previous upper-bound point,
    // then spread seeds across the enlarged region so the initial
    // design can separate Kp, Ki, and Kd effects.
    {2.0,      0.0,   0.20},
    {100.0,   50.0,  20.00},
    {250.0,  400.0,  80.00},
    {400.0,  150.0, 160.00},
    {550.0,  500.0,  40.00},
    {700.0,  250.0, 200.00},
    {850.0,  100.0, 120.00},
    {1000.0, 450.0, 180.00},
    {1000.0, 500.0, 200.00}
};'''
replace_once("jmp/DiscoveryPID_Position_V2.jsl", old_seeds, new_seeds)

replace_once(
    "scripts/position_v2/open_jmp.ps1",
    'data\\position_autotune_directpid_v37_300degwide\\sessions',
    'data\\position_autotune_directpid_v38_300deg10xrange\\sessions',
)

cfg_path = Path("config/discoverypid.json")
cfg = json.loads(cfg_path.read_text(encoding="utf-8"))
cfg["profileName"] = "Direct PID / 300-degree Bayesian study - 10x PID range"
cfg["firmwareId"] = "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange 2026-08-18"
cfg["studyName"] = "PositionStudy_DirectPID_v38_300Deg10xRange"
cfg["pidRanges"] = {
    "kp": [0.5, 1000.0],
    "ki": [0.0, 500.0],
    "kd": [0.0, 200.0],
}
cfg_path.write_text(json.dumps(cfg, indent=2) + "\n", encoding="utf-8")

replace_once(
    "README.md",
    "DiscoveryPID direct-position-pid-v3.3-300deg 2026-08-17",
    "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange 2026-08-18",
)
replace_once("README.md", "- Kp search: **0.5 to 100**", "- Kp search: **0.5 to 1000**")
replace_once("README.md", "- Ki search: **0 to 50**", "- Ki search: **0 to 500**")
replace_once("README.md", "- Kd search: **0 to 20**", "- Kd search: **0 to 200**")

replace_once(".github/workflows/ci.yml", "assert cfg['pidRanges']['kp'] == [0.5, 100.0]", "assert cfg['pidRanges']['kp'] == [0.5, 1000.0]")
replace_once(".github/workflows/ci.yml", "assert cfg['pidRanges']['ki'] == [0.0, 50.0]", "assert cfg['pidRanges']['ki'] == [0.0, 500.0]")
replace_once(".github/workflows/ci.yml", "assert cfg['pidRanges']['kd'] == [0.0, 20.0]", "assert cfg['pidRanges']['kd'] == [0.0, 200.0]")
replace_once(
    ".github/workflows/ci.yml",
    "assert 'direct-position-pid-v3.3-300deg' in fw",
    "assert 'direct-position-pid-v3.4-300deg-10xrange' in fw\n          assert 'MAX_POSITION_KP = 1000.0f' in fw\n          assert 'MAX_POSITION_KI = 500.0f' in fw\n          assert 'MAX_POSITION_KD = 200.0f' in fw",
)
replace_once(
    ".github/workflows/ci.yml",
    'assert \'EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.3-300deg"\' in py',
    'assert \'EXPECTED_FW = "DiscoveryPID direct-position-pid-v3.4-300deg-10xrange"\' in py',
)
replace_once(".github/workflows/ci.yml", "assert 'kpHigh = 100.0;' in jsl", "assert 'kpHigh = 1000.0;' in jsl")
replace_once(".github/workflows/ci.yml", "assert 'kiHigh = 50.0;' in jsl", "assert 'kiHigh = 500.0;' in jsl")
replace_once(".github/workflows/ci.yml", "assert 'kdHigh = 20.0;' in jsl", "assert 'kdHigh = 200.0;' in jsl")
