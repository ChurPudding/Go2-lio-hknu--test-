# =============================================================================
# zupt_ab_env.sh  —  run_zupt_ab_batch.sh · zupt_ab_summarize.sh 공용 설정 (source 해서 씀)
# -----------------------------------------------------------------------------
#   AB_KEY : A/B 로 토글할 키 하나. mapping_go2_fix.launch.py 의 선택 인자
#            (<키>:=true|false)로 넘기고 go2_fix.yaml 은 고치지 않는다.
#            다른 키로 돌릴 때 :  AB_KEY=zupt_en ./run_zupt_ab_batch.sh
#   출력·로그는 키별 하위 폴더로 나눈다 (키를 바꿔도 이전 결과를 건너뛰거나 덮어쓰지 않게).
#   2026-10-01 이전 zupt_en 결과(폴더 없이 바로 아래)를 다시 집계할 때 :
#     OUTDIR=~/data/bags LOGDIR=~/data/zupt_ab_logs ./zupt_ab_summarize.sh
# =============================================================================
AB_KEY="${AB_KEY:-leg_en}"
BAGDIR=~/data/bags                                    # 입력 bag
OUTDIR="${OUTDIR:-$BAGDIR/ab_${AB_KEY}}"              # 출력 bag : plout_run{1,2}_{off,on}_r{1..3}
LOGDIR="${LOGDIR:-$HOME/data/zupt_ab_logs/${AB_KEY}}" # 런 로그 · summary.csv
