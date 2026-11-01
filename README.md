# aichallenge-racingkart

本リポジトリでは、自動運転AIチャレンジでご利用いただく開発環境を提供します。参加者の皆様には、Autoware Universe をベースとした自動運転ソフトウェアを開発し、予選大会では End to End シミュレーション空間を走行するレーシングカートにインテグレートしていただきます。開発した自動運転ソフトウェアで、安全に走行しながらタイムアタックに勝利することが目標です。また、決勝大会では本物のレーシングカートへのインテグレーションを行っていただきます。

This repository provides a development environment use in the Automotive AI Challenge. For the preliminaries, participants will develop autonomous driving software based on Autoware Universe and integrate it into a racing kart that drives in the End to End simulation space. The goal is to win in time attack while driving safely with the developed autonomous driving software. Also, for the finals, qualifiers will integrate it into a real racing kart.
---

## Repository introduction

### Overview

This repository was used by [Thang Le](https://github.com/ThangLe-VictorLe) during the JSAE AI Challenge 2026.

### Team

- **Team name**: `sudo rm -rf`
- **Development period**: 04/08/2026 - 01/09/2026
- **Team members**: [Harry Nguyen](https://www.linkedin.com/in/harringuyen/)

### Technical Approach

The controller was based on Model Predictive Control (MPC) using OSQP solver in Fernet frame running at 40Hz

We worked on many areas of autonomous racing, including:
- MPC controller, Vehicle to Everything (V2X) implement and parameter tuning
- Kart default trajectory modification
- Dynamic opponent profiling, record opponent trajectory
- Steering gain and velocity control
- Overtaking procedure: 4 tier side choosing system, Side memory
- Autonomous vehicle recovery: different states, stuck detection, reverse steering, cooldown for re-entry
- Computing cost optimization: save precomputed values, hot-swapping mpc sight, margin relaxtion
- Debugging and improving the  system through online battle simulation

The project involved working with technologies and tools such as `ROS 2`, `Autoware Universe`, `AWSIM`, `Docker`, and `RViz2`. For more detailed information about Technical Approach and development process, please take a look at [TECHNICAL_APPROACH_AND_DEVELOPMENT_PROCESS.md](TECHNICAL_APPROACH_AND_DEVELOPMENT_PROCESS.md) in this repo

### Development Process

Our development process is all about the cycles of implementation, simulation, evaluation, then debugging. We used simulation footages, logs, and `RViz2` visualizations to investigate controller behavior.

```bash
# Phase 1: Raceline modification, steer gain and velocity tunning & benchmark lap time
# Phase 2: Crash Recovery & Fail-safes
# Phase 3: Performance & OSQP Overhaul
# Phase 4: Overtaking Logic
# Phase 5: Edge-Case Tuning & Safety Gates clearance
```

The repository's Git commit history also records the progression of our development, including implementation new features and debugging.

### Competition Results

Our team reached our highest transition score in the qualifying round, and peaked at 4th place in the student division.

On the last qualifying day, we eventually finished at 17th place in the student division, just outside the top 16 qualifying cutline. Even though, we did not get into the finals, the challenge taught us many valuable practical experiences in autonomous systems, control engineering, and robotics software development.

### Repository Guide

- **Organizer-provided environment**: See the original repository structure and official documentation below.
- **Team-developed code**: Most of the racing kart's logic resides in [`aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/`](aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/). The primary files edited are:
  - [`mpc_controller.py`](aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/mpc_controller.py): `multi_purpose_mpc_ros/mpc_controller.py`
  - [`reference_path.py`](aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/reference_path.py): `multi_purpose_mpc_ros/core/reference_path.py`
  - [`MPC.py`](aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/core/MPC.py): `multi_purpose_mpc_ros/core/MPC.py`
  - [`config.yaml`](aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/config/config.yaml): `config/config.yaml`
  - [`v2x_vehicle_tracker.py`](aichallenge/workspace/src/aichallenge_submit/multi_purpose_mpc_ros/multi_purpose_mpc_ros/v2x_vehicle_tracker.py): `multi_purpose_mpc_ros/v2x_vehicle_tracker.py`
  - Plus various supporting logic files inside `aichallenge_submit/`
- **Development history**: Browse the Git commit history to follow the project's evolution.

For environment setup and competition-related instructions, please refer to the official documentation linked in the original README.

### Acknowledgements

We would like to thank the JSAE AI Challenge organizers for the development environment and competition platform, as well as everyone involved in making the challenge possible.

This repository is based on the organizer-provided aichallenge-racingkart project. Please check out the original project and its license for the licensing terms and attribution of the underlying environment. 

Original organizer repository: [https://github.com/AutomotiveAIChallenge/aichallenge-documentation-racingkart](https://github.com/AutomotiveAIChallenge/aichallenge-documentation-racingkart)

---

## ドキュメント / Documentation

ルールの詳細や環境構築方法など、大会に関する情報を以下のページで提供します。

Toward the competition, we will update the following pages to provide information such as rules and how to set up your dev environment. Please follow them. We are looking forward your participation!

- [日本語ページ](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/)
- [English Page](https://automotiveaichallenge.github.io/aichallenge-documentation-racingkart/en/)
- [スクリプト設計メモ（評価/ビルド/起動）](aichallenge/README.md)

## リポジトリ構成（トップレベル）

- `aichallenge/`: シミュレータ/Autoware/評価の起動・操作スクリプト群
- `vehicle/`: 実車環境向け（セットアップ確認、Zenoh、rosbag など）
- `remote/`: 実車/遠隔接続の補助（SSH/Zenoh/RViz/joy）
- `docs/`: ガイド・スライドは [`docs/guide/`](docs/guide/)、仕様は [`docs/spec/`](docs/spec/)、インターフェイス契約は [`docs/interface/`](docs/interface/)
- `submit/`: 提出物（tar.gz）置き場
- `output/`: 実行結果・ログ出力先（生成物）

## Docker Compose（推奨）

### 全体像（開発: Makefile / 個別起動）

```text
Host (you)
  ├─ make autoware-build / make dev / make eval / make simulator ...
  └─ docker compose（.env の COMPOSE_FILE でオーバーレイを選択）
        ├─ docker-compose.yml       (ベース: 全サービス定義)
        │     ├─ autoware           (Autoware)
        │     ├─ autoware-build     (colcon build 専用)
        │     ├─ simulator          (AWSIM)
        │     ├─ autoware-command   (ros2 service/topic の単発操作)
        │     ├─ zenoh              (Zenoh ブリッジ)
        │     ├─ driver             (実車ドライバ)
        │     └─ rviz2              (可視化)
        ├─ docker-compose.gpu.yml   (NVIDIA GPU オーバーレイ、オプション)
        └─ docker-compose.sound.yml (PulseAudio / simulator のみ、オプション)
        → output/ にログ・結果を出力（最新結果は output/latest/d<domain>/）
```

オーバーレイの選択は `.env` の `COMPOSE_FILE` 行で行う（`docker compose` が自動読み込み）。
`./setup.bash env` を実行すると GPU の有無を自動検出して `.env` を生成する。

### はじめてのセットアップ

```bash
# 1. 環境セットアップ（Docker インストール〜.env 生成〜イメージ取得〜AWSIM DL）
./setup.bash bootstrap

# または個別に実施する場合:
./setup.bash env            # .env 生成（GPU/CPU 自動判定）
./setup.bash network tune   # DDS ホストチューニング（rmem_max + loマルチキャスト）
./setup.bash download awsim # AWSIM バイナリのダウンロード
./setup.bash pull image     # Autoware ベースイメージの取得

# 2. 開発イメージのビルド
./docker_build.sh dev

# 3. ROS 2 ワークスペースのビルド（コンテナ内 colcon）
make autoware-build

# 4. 開発ループ（AWSIM + Autoware 起動）
make dev
make autoware-request-initialpose
make autoware-request-control
make down
```

コンテナはホストの UID/GID（`HOST_UID`/`HOST_GID`）で動作するため、`output/` 配下の生成物はホストユーザー所有になり root 権限は不要です。

## まずは読んでほしいもの

- [初学者向けセットアップ資料](./docs/spec/how-to-setup.md)
- [初学者向け説明資料](./docs/spec/introduction.md)
- [初学者向けリポジトリ入門スライド (Marp)](./docs/guide/beginner-deck.marp.md)

## OSS 貢献にあたって

PR を出す前に `pre-commit run -a` を通してください。

```.sh
check for merge conflicts................................................Passed
check xml................................................................Passed
check yaml...............................................................Passed
detect private key.......................................................Passed
fix end of files.........................................................Passed
mixed line ending........................................................Passed
shellcheck...............................................................Passed
shfmt....................................................................Passed
```
