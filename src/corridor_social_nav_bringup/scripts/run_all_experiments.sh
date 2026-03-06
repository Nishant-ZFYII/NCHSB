#!/bin/bash
#
# Master experiment runner for corridor social navigation.
#
# Executes the full experiment matrix:
#   Main comparison:   4 controllers x 6 scenarios x 30 seeds  = 720 trials
#   Ablation 1 (T):    4 prediction horizons x 30 seeds        = 120 trials
#   Ablation 2 (frame): 2 configs x 3 scenarios x 30 seeds     = 180 trials
#   Ablation 3 (shape): 2 configs x 30 seeds                   =  60 trials
#   Ablation 4 (type):  2 configs x 30 seeds                   =  60 trials
#   Ablation 5 (shield): 2 configs x 6 scenarios x 30 seeds    = 360 trials
#   Ablation 6 (margin): 3 configs x 30 seeds                  =  90 trials
#   Ablation 7 (noise):  4 configs x 30 seeds                  = 120 trials
#   ─────────────────────────────────────────────────────────────────
#   TOTAL                                                      ~1,710 trials
#   Estimated runtime: ~85 hours at 3 min/trial
#
# Usage:
#   ./run_all_experiments.sh [--output_dir DIR] [--start_phase N] [--dry_run]
#
# Phases:
#   1 = Main comparison (720 trials)
#   2 = Ablation studies (990 trials)
#   3 = Both

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
SCENARIOS_DIR="$PKG_DIR/scenarios"
RUNNER="$SCRIPT_DIR/run_experiment.py"

OUTPUT_DIR="${1:-results}"
START_PHASE="${2:-1}"
DRY_RUN="${3:-false}"

SEEDS_MIN=0
SEEDS_MAX=29

SCENARIOS=(
    head_on_single
    head_on_group
    doorway_popout
    junction_crossing
    overtaking
    bidirectional
)

CONTROLLERS=(
    mppi_vanilla
    mppi_social
    mppi_pred
    full
)

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$OUTPUT_DIR/logs_$TIMESTAMP"
SUMMARY_FILE="$OUTPUT_DIR/experiment_summary_$TIMESTAMP.csv"
mkdir -p "$LOG_DIR"

TRIAL_COUNT=0
FAIL_COUNT=0

log() {
    echo "[$(date '+%H:%M:%S')] $*"
}

run_trial() {
    local scenario=$1
    local controller=$2
    local seed=$3
    local extra_args="${4:-}"

    TRIAL_COUNT=$((TRIAL_COUNT + 1))
    local trial_id="${scenario}_${controller}_seed${seed}"
    local log_file="$LOG_DIR/${trial_id}.log"

    if [ "$DRY_RUN" = "true" ]; then
        log "[DRY RUN] Trial #$TRIAL_COUNT: $trial_id"
        return 0
    fi

    log "Trial #$TRIAL_COUNT: $trial_id"

    local scenario_file="$SCENARIOS_DIR/${scenario}.yaml"
    if [ ! -f "$scenario_file" ]; then
        log "  [ERROR] Scenario file not found: $scenario_file"
        FAIL_COUNT=$((FAIL_COUNT + 1))
        return 1
    fi

    python3 "$RUNNER" \
        --scenario "$scenario_file" \
        --controller "$controller" \
        --seed "$seed" \
        --output_dir "$OUTPUT_DIR/data" \
        $extra_args \
        > "$log_file" 2>&1

    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        log "  [FAIL] Trial exited with code $exit_code"
        FAIL_COUNT=$((FAIL_COUNT + 1))
    else
        log "  [OK] Trial completed"
    fi

    # Brief pause between trials for cleanup
    sleep 5

    return $exit_code
}

# ═══════════════════════════════════════════════
# PHASE 1: Main Comparison (720 trials)
# ═══════════════════════════════════════════════
run_main_comparison() {
    log "═══════════════════════════════════════════════"
    log "PHASE 1: Main Comparison"
    log "  4 controllers x 6 scenarios x 30 seeds = 720 trials"
    log "═══════════════════════════════════════════════"

    for scenario in "${SCENARIOS[@]}"; do
        for controller in "${CONTROLLERS[@]}"; do
            for seed in $(seq $SEEDS_MIN $SEEDS_MAX); do
                run_trial "$scenario" "$controller" "$seed" || true
            done
        done
    done
}

# ═══════════════════════════════════════════════
# PHASE 2: Ablation Studies (990 trials)
# ═══════════════════════════════════════════════
run_ablations() {
    log "═══════════════════════════════════════════════"
    log "PHASE 2: Ablation Studies"
    log "═══════════════════════════════════════════════"

    # Ablation 5: Shield on/off across all scenarios (360 trials)
    # Uses mppi_pred (no shield) vs full (with shield)
    log "--- Ablation 5: Shield on/off (360 trials) ---"
    for scenario in "${SCENARIOS[@]}"; do
        for controller in mppi_pred full; do
            for seed in $(seq $SEEDS_MIN $SEEDS_MAX); do
                run_trial "$scenario" "$controller" "$seed" || true
            done
        done
    done

    # Ablation 2: Corridor frame vs global frame (180 trials)
    # Run across 3 scenarios: head_on_single, junction_crossing, bidirectional
    log "--- Ablation 2: Frame comparison (180 trials) ---"
    for scenario in head_on_single junction_crossing bidirectional; do
        for controller in mppi_pred full; do
            for seed in $(seq $SEEDS_MIN $SEEDS_MAX); do
                run_trial "$scenario" "$controller" "$seed" || true
            done
        done
    done

    # Ablation 7: Tracking noise (120 trials)
    log "--- Ablation 7: Tracking noise (120 trials) ---"
    for noise in 0.0 0.05 0.1 0.2; do
        for seed in $(seq $SEEDS_MIN $SEEDS_MAX); do
            run_trial "head_on_single" "full" "$seed" \
                "--tracking_noise_std $noise" || true
        done
    done
}

# ═══════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════

log "Corridor Social Navigation Experiment Suite"
log "Output: $OUTPUT_DIR"
log "Phase: $START_PHASE"
log ""

echo "scenario,controller,seed,status,elapsed_s" > "$SUMMARY_FILE"

case "$START_PHASE" in
    1)
        run_main_comparison
        ;;
    2)
        run_ablations
        ;;
    3|*)
        run_main_comparison
        run_ablations
        ;;
esac

log ""
log "═══════════════════════════════════════════════"
log "EXPERIMENT SUITE COMPLETE"
log "  Total trials: $TRIAL_COUNT"
log "  Failed: $FAIL_COUNT"
log "  Summary: $SUMMARY_FILE"
log "  Logs: $LOG_DIR/"
log "═══════════════════════════════════════════════"
