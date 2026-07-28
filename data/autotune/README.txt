Generated JMP autotuning data is stored here.

Expected generated files:

    DiscoveryPID_Optimization.jmp
    DiscoveryPID_Optimization.csv
    DiscoveryPID_Response_History.csv
    runs/Run_###_response.csv
    results/Run_###_result.csv

The compact optimization CSV contains one completed row per physical run:

    P,I,Error

Error is the smoothed mean absolute target-tracking error calculated by
tools/run_pid_trial.py after the configured warm-up period.
