# xArm6 鍥涚墿浣撶幇鍦哄懡浠ゅ崱

杩欏紶鍛戒护鍗℃寜鐪熷疄鎵ц椤哄簭鎺掑垪銆俙[OFFLINE]` 鍙鏈湴鏂囦欢锛沗[ROBOT]` 浼氳繛鎺ュ苟杩愬姩 xArm6锛屽繀椤荤敱鐜板満
鎿嶄綔鑰呯‘璁よ蒋鍨€佹€ュ仠鍜屽噣绌哄悗鎵嬪姩杩愯銆傜湡鏈虹數鑴戣嫢宸叉縺娲?Python 鐜锛岀粺涓€浣跨敤 `python`銆?
## 0. 寮€鏈哄悗鍏堝仛绂荤嚎棰勬

```bash
# [OFFLINE] 妫€鏌?Python/SDK銆乭ardware config銆佸洓鐗╀綋 profile 鍜?G1 鏍囧畾鐘舵€?python scripts/28_check_real_robot_environment.py \
  --output real_handoff/onsite_environment.json

# [OFFLINE] 閲嶆柊鐢熸垚鍥涚墿浣?handoff 鎶ュ憡
python scripts/27_check_four_object_handoff.py \
  --output real_handoff/four_object_plan_check.json
```

棰勬蹇呴』鏄剧ず Python銆乣xarm-python-sdk`銆丯umPy銆丼ciPy銆乭ardware config 鍜屽洓鐗╀綋鏂囦欢鍧囦负 `PASS`銆侽0 G1
搴斾负 `PASS`锛汷1鈥揙3 鍦ㄦ爣瀹氬墠鏄剧ず `WAIT` 灞炰簬姝ｅ父鐘舵€併€傞妫€鏈韩涓嶄細瀵煎叆 SDK鎴栬繛鎺ユ満鍣ㄤ汉銆?
## 1. 褰撳ぉ鍏堟仮澶?O0 淇濆簳缁撴灉

```bash
# [OFFLINE] 妫€鏌ヨ鍒?python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json

# [ROBOT] 绌鸿噦閫熷害姊害
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --speed-scale 0.25 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --speed-scale 0.5 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json \
  --speed-scale 1.0 --execute-empty-arm

# [ROBOT] 绌?G1銆佽蒋鍨姏鍑恒€佹渶鍚庢墠瀹屾暣鎺ュ彇
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json --execute-empty-g1
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json --execute-throw-only
python scripts/24_run_cube_open_loop_demo.py \
  --profile configs/open_loop_flip/cube38/low_5deg.json --execute-cube
```

O0 low 鎺ヤ綇鍚庣珛鍒讳繚瀛樻棩蹇楀拰淇濆簳瑙嗛銆傚綋澶╀笉鍏堣拷澶ц搴︺€?
## 2. O1鈥揙3 閫愮墿浣撴爣瀹?
| Object | Template | Calibrated output stem |
|---|---|---|
| O1 | `configs/open_loop_flip/cuboid30/low_3deg.json` | `cuboid30/low` |
| O2 | `configs/open_loop_flip/cuboid33/low_5deg.json` | `cuboid33/low` |
| O3 | `configs/open_loop_flip/cuboid38/low_4p5deg.json` | `cuboid38/low` |

瀵瑰綋鍓嶇墿浣撳疄娴?`<HELD> <RELEASE> <PRECLOSE> <CLOSE>` 鍚庯紝鍏堢敓鎴?`empty_g1`銆備笅闈互 O1 涓轰緥锛?
```bash
# [OFFLINE]
python scripts/26_calibrate_open_loop_profile.py \
  --template-profile configs/open_loop_flip/cuboid30/low_3deg.json \
  --output-profile configs/open_loop_flip/real_calibrated/cuboid30/low_empty_g1.json \
  --output-schedule real_handoff/cuboid30/low/g1_schedule.empty_g1.json \
  --held-position <HELD> --release-position <RELEASE> \
  --preclose-position <PRECLOSE> --close-position <CLOSE> \
  --stage empty_g1
```

O0 鐨勫巻鍙叉暣鏁颁綅缃彧鑳戒綔涓?O0 璧风偣锛屼笉鑳藉鍒剁粰 O1鈥揙3銆傚畬鎴?empty G1 鍚庯紝鐢ㄥ悓涓€缁勫疄娴嬪€煎垎鍒敓鎴?`throw_only` 鍜?`object`锛屾枃浠跺悕涓?`--stage` 鍚屾鏀瑰彉锛?
```bash
# [OFFLINE] 绀轰緥锛氭妸 STAGE 渚濇鏇挎崲涓?throw_only銆乷bject
python scripts/26_calibrate_open_loop_profile.py \
  --template-profile configs/open_loop_flip/cuboid30/low_3deg.json \
  --output-profile configs/open_loop_flip/real_calibrated/cuboid30/low_STAGE.json \
  --output-schedule real_handoff/cuboid30/low/g1_schedule.STAGE.json \
  --held-position <HELD> --release-position <RELEASE> \
  --preclose-position <PRECLOSE> --close-position <CLOSE> \
  --stage STAGE
```

## 3. 姣忎釜鏂扮墿浣撶殑鎵ц姊害

涓嬮潰渚濇浣跨敤鍒氱敓鎴愮殑 `<EMPTY_G1_PROFILE>`銆乣<THROW_ONLY_PROFILE>` 鍜?`<OBJECT_PROFILE>`锛?
```bash
# [OFFLINE]
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE>

# [ROBOT] 绌鸿噦 0.25x 鈫?0.5x 鈫?1.0x
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --speed-scale 0.25 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --speed-scale 0.5 --execute-empty-arm
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --speed-scale 1.0 --execute-empty-arm

# [ROBOT] G1 鈫?杞灚鎶涘嚭 鈫?瀹屾暣鎺ュ彇
python scripts/24_run_cube_open_loop_demo.py --profile <EMPTY_G1_PROFILE> \
  --execute-empty-g1
python scripts/24_run_cube_open_loop_demo.py --profile <THROW_ONLY_PROFILE> \
  --execute-throw-only
python scripts/24_run_cube_open_loop_demo.py --profile <OBJECT_PROFILE> \
  --execute-object
```

涓ユ牸鎸?O1 鈫?O2 鈫?O3 鎺ㄨ繘銆傛瘡瀹屾垚涓€涓墿浣撳氨淇濆瓨鎴愬姛 profile銆丟1鏁存暟銆佹棩蹇楀拰瑙嗛锛屼笉绛夊埌鏈€鍚庣粺涓€鏁寸悊銆?
## 4. 鍥涚墿浣撴垚鍔熷悗鍐嶅仛 pose 姊害

| Object | Low | Next pose | High |
|---|---|---|---|
| O0 | `cube38/low_5deg.json` | `cube38/medium_6p5deg.json` | `cube38/high_8deg.json` |
| O1 | `cuboid30/low_3deg.json` | `cuboid30/pose_conditioned_5p5deg.json` | `cuboid30/high_6p5deg.json` |
| O2 | `cuboid33/low_5deg.json` | `cuboid33/pose_conditioned_5p5deg.json` | `cuboid33/high_6p5deg.json` |
| O3 | `cuboid38/low_4p5deg.json` | `cuboid38/pose_conditioned_5p5deg.json` | `cuboid38/high_6p5deg.json` |

姣忔潯鏂?pose 閮介噸鏂拌蛋 plan-only銆佺┖鑷傘€佺┖ G1銆乼hrow-only銆乷bject 姊害銆傛姤鍛婅搴﹀彇渚ц瑙嗛瀹炴祴鍊硷紱profile
鍚嶇О涓殑鐩爣瑙掑彧琛ㄧず policy 杈撳叆銆傚洓鐗╀綋鍚勪竴涓畬鏁存垚鍔熶箣鍓嶏紝涓嶆姇鍏ユ椂闂磋拷 20掳浠ヤ笂銆?
## 5. 姣忔杩愯椹笂璁板綍

```text
object / profile / held / release / preclose / close
鏄惁瀹屽叏绂绘墜 / 瀹炴祴鏃嬭浆瑙?/ 鏄惁鎺ヤ綇 / 淇濇寔鏃堕棿
normal-speed video / slow-motion video / output summary path
```

鍑虹幇 tracking error銆佸紓甯告尟鍔ㄣ€丟1/cable 骞叉秹鎴栫墿浣撻鍑鸿蒋鍨寖鍥存椂锛屽仠姝㈠綋鍓?profile锛屼繚鐣欐棩蹇楋紝鍥炲埌绂荤嚎
璋冩暣銆備笉瑕佸湪鐜板満涓存椂鍗囩骇 SDK锛屼篃涓嶈璺宠繃浣庨€熺┖鑷傞樁娈点€?
