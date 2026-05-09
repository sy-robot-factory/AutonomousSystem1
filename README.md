# Autonomous System 1

本プロジェクトは、エージェントプログラミングを学ぶための教育用シミュレータです。
5台のエージェントを制御し、未知の環境で効率よく資源を回収するアルゴリズムを実装しましょう。

## プロジェクトの目的
5台のエージェントが協力（あるいは個別に最適化）し、制限時間内に獲得した**資源の総量**を最大化することを目指します。

## 基本ルール
- **資源の獲得**: エージェントの中心座標が資源（円形）の内部に入ると、自動的に資源を獲得します。
- **資源の性質**: 資源は有限量です。0になるとその場所から消滅し、別のランダムな場所に再出現します。
- **環境のランダム性**: 5台のエージェントの初期位置および障害物の配置は、実行ごとにランダムに決定されます。
- **行動決定**: 一定時間（シミュレーションステップ）ごとにセンサ情報が更新され、各エージェントは次の行動を決定します。

## エージェントの仕様
エージェントには以下のセンサと通信能力が備わっています。

1.  **距離センサ**:
    - センサ範囲は5つのセグメントに等分されています。
    - 各範囲ごとに、範囲内のオブジェクトの「種類」「距離」「追加情報」を取得できます。
2.  **衝突センサ**:
    - 他のエージェント、または障害物との衝突を検知します。
3.  **通信範囲**:
    - 一定の範囲内にいる他のエージェントと情報を共有することが可能です。

## 環境構築と実行方法

本プロジェクトでは、Pythonのパッケージ管理ツール [uv](https://docs.astral.sh/uv/) を使用した実行を推奨しています。

### 1. リポジトリのクローン
まずはターミナル（またはコマンドプロンプト）を開き、プロジェクトをクローンします。
```bash
git clone https://github.com/sy-robot-factory/AutonomousSystem1.git
cd AutonomousSystem1
```

### 2. シミュレータの起動
`uv` がインストールされている場合、以下のコマンドを実行するだけで、必要なライブラリ（pyglet, numpy等）のインストールが自動的に行われます。
```bash
uv sync
```
`uv`をインストールしていない場合は先に
```bash
pip install uv
```
でインストールしてください．

シミュレータを起動する際は以下のコマンドを実行します．
```bash
uv run main.py
```
※ `uv` を使用しない場合は、`pip install pyglet numpy` を実行した後に `python main.py` で起動してください。

## 実装方法
受講者の皆さんは、`material_collecting_agent.py`の`MaterialCollectingAgent` を継承した新しいエージェントクラスを作成してください。
実装のサンプルとして `my_material_collecting_agent.py` が用意されています。このファイルでは`MaterialCollectingAgent`を継承した`MyMaterialCollectingAgent`が実装されています。


### 実装のポイント
- **必須実装**: `act` メソッド内に、センサ情報に基づいた行動ロジックを記述してください。
- **内部変数**: 状態を保持するためのフィールド（変数）は、自身のサブクラス内に自由に追加して構いません。
- **禁止事項**: `agent.py` 本体や、その他のシステムクラス（シミュレータの基幹部分）は変更しないでください。

## リファレンス

エージェントの行動決定は、`self.params` オブジェクトを読み書きすることで行います。

### 1. センサ情報の取得 (読み取り専用)
| 変数名 | 型 | 内容 |
| :--- | :--- | :--- |
| `self.params.sensor_object_type` | ndarray(5) | 5つのセグメントごとのオブジェクト種別 (0:なし, 1:エージェント, 2:障害物, 3:資源) |
| `self.params.sensor_object_distance` | ndarray(5) | 5つのセグメントごとのオブジェクトまでの距離 |
| `self.params.sensor_object_attribute` | ndarray(5) | オブジェクトの追加情報（資源の量など） |
| `self.params.collision_sensor` | int | 衝突状態 (`MaterialCollectingAgentParameters.SENSE_COLLIDED` で判定) |
| `self.params.received_messages` | list[str] | 通信範囲内にいる他エージェントからのメッセージリスト |

### 2. オブジェクトの追加情報
| 検知対象 (`sensor_object_type`) | 格納される値 (`sensor_object_attribute`) |
| :--- | :--- |
| `SENSE_MATERIAL` (3) | 検知したマテリアルの**半径 (radius)** |
| `SENSE_AGENT` (1) | 検知した他エージェントの**ID** |
| `SENSE_OBSTACLE` (2) | 0 (固定) |
| `SENSE_NONE` (0) | 0 (固定) |

### 3. 行動の設定 (書き込み)
`self.params.action` に以下の定数を代入することで行動を選択します。

| アクション定数 | 説明 | 必要な追加パラメータ |
| :--- | :--- | :--- |
| `ACT_GO_FORWARD` | 前進 | `self.params.velocity` (-MAX_VELOCITY～MAX_VELOCITY) |
| `ACT_ROTATE` | 回転 | `self.params.angular_velocity` (-MAX_ANGULAR_VELOCITY～MAX_ANGULAR_VELOCITY) |
| `ACT_STANDSTILL` | 停止 | なし |

### 4. 通信の設定 (書き込み)
- `self.params.communication_message`: 文字列を代入すると、次ステップで周囲のエージェントに送信されます。

### 実装例 (actメソッド内)
```python
def act(self):
    # 1. センサ情報の確認
    if self.params.collision_sensor == MaterialCollectingAgentParameters.SENSE_COLLIDED:
        # 衝突していたら回転して回避
        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
        self.params.angular_velocity = 45
    else:
        # 正面に資源があるかチェック (中央のセンサはインデックス2)
        if self.params.sensor_object_type[2] == 3: 
            # 資源があれば前進
            self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY
        else:
            # 探索のためにランダムに動く
            self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
            self.params.angular_velocity = random.uniform(-30, 30)

    # メッセージの更新
    self.params.communication_message = f"Agent {self.id} searching..."
```

## 実験の実行と設定
`main.py` を実行することでシミュレーションが開始されます。

### main.py の主要パラメータ
実験を効率的に進めるために、以下の変数を適宜変更して調整してください。
- `auto_executor`: `True` に設定すると自動でシミュレーションを繰り返します。
- `double_speed_mode`: `True` に設定すると、描画速度を上げて高速に実験を行えます。

### 実験記録とCSVファイルの見方
シミュレーション終了後、実行ディレクトリに実験データがCSVファイルとして保存されます。このデータをExcelやPythonのpandasなどで読み込むことで、アルゴリズムの性能を定量的に評価できます。

- **ファイル名**: `log_YYYYMMDD_HHMMSS.csv` (実行時の日時が反映されます)
- **行**:
    - `step`: シミュレーション開始からの経過時間（ステップ数）。

- **列**:
    - [全エージェントの獲得総量], [ID0の獲得量], [ID1の獲得量], …, [ID5の獲得量]
    

**分析のヒント**:
- グラフの横軸を `step`、縦軸を全エージェントの獲得総量にするなどしてプロットし、より多くの資源が獲得できる制御を目指しましょう。
- 特定のエージェントだけ資源獲得量が多い（または少ない）場合、探索範囲の偏りや、衝突回避ロジックの問題がないかを確認してください。

---

### クイックスタート
1. `my_material_collecting_agent.py` をコピーして新しいファイルを作成する。
2. `act` メソッドを書き換えて、独自のスキャン・移動ロジックを実装する。
3. `main.py` を編集して、作成したエージェントをシミュレータに登録する（下記参照）。
4. `main.py` を実行する。

#### 自分のエージェントを main.py に反映させる方法
作成したエージェントクラス（例：`OriginalAgent`）を反映させるには、`main.py` を以下のように修正してください。

1. **インポートの追加**:
   `main.py` の冒頭（7行目付近）で、自分のファイルからクラスを読み込むように変更します。
   ```python
   from original_agent_file import OriginalAgent  # 自分のファイル名とクラス名
   ```

2. **クラス名の差し替え**:
   `FrameManager` クラスの `reset_environment` メソッド内（41行目付近）のエージェント生成部分を書き換えます。
   ```python
   for i in range(num_agent):
       mc_agent = OriginalAgent()  # ここを自分のクラス名に変更
       mc_env.register_agent(mc_agent)
   ```
---
# English version:
# Autonomous System 1

This project is an educational simulator designed for learning agent programming.
Implement algorithms to control five agents and efficiently collect resources in an unknown environment.

## Project Objective
The goal is for five agents to cooperate (or optimize individually) to maximize the **total amount of resources** collected within the time limit.

## Basic Rules
- **Resource Collection**: An agent automatically collects a resource when its center coordinates enter the resource's circular area.
- **Resource Properties**: Resources are finite. When a resource reaches zero, it disappears and reappears in another random location.
- **Environmental Randomness**: The initial positions of the five agents and the placement of obstacles are determined randomly for each run.
- **Decision Making**: Sensor information is updated at regular intervals (simulation steps), and each agent decides its next action.

## Agent Specifications
Agents are equipped with the following sensors and communication capabilities:

1.  **Distance Sensor**:
    - The sensor range is divided into five equal segments.
    - For each segment, you can retrieve the "Type," "Distance," and "Additional Information" of objects within range.
2.  **Collision Sensor**:
    - Detects collisions with other agents or obstacles.
3.  **Communication Range**:
    - Agents can share information with other agents within a certain range.

## Setup and Execution

We recommend using the Python package management tool [uv](https://docs.astral.sh/uv/) for this project.

### 1. Clone the Repository
Open your terminal (or command prompt) and clone the project:
```bash
git clone https://github.com/sy-robot-factory/AutonomousSystem1.git
cd AutonomousSystem1
```

### 2. Launch the Simulator
If you have `uv` installed, simply run the following command to automatically install the necessary libraries (pyglet, numpy, etc.):
```bash
uv sync
```
If you don't have `uv` installed, please install it first via:
```bash
pip install uv
```

To start the simulator, run:
```bash
uv run main.py
```
*Note: If you are not using `uv`, run `pip install pyglet numpy` and then start with `python main.py`.*

## Implementation Guide
Participants should create a new agent class that inherits from `MaterialCollectingAgent` in `material_collecting_agent.py`.
An implementation example is provided in `my_material_collecting_agent.py`, where `MyMaterialCollectingAgent` is defined.

### Key Implementation Points
- **Mandatory**: Write your action logic based on sensor information within the `act` method.
- **Internal Variables**: You are free to add fields (variables) to your subclass to maintain state.
- **Prohibitions**: Do not modify the core `agent.py` file or other system classes (the simulator's core engine).

## Reference

Agent behavior is determined by reading and writing to the `self.params` object.

### 1. Getting Sensor Information (Read-only)
| Variable Name | Type | Description |
| :--- | :--- | :--- |
| `self.params.sensor_object_type` | ndarray(5) | Object type for each segment (0:None, 1:Agent, 2:Obstacle, 3:Resource) |
| `self.params.sensor_object_distance` | ndarray(5) | Distance to the object in each segment |
| `self.params.sensor_object_attribute` | ndarray(5) | Additional object info (e.g., resource amount) |
| `self.params.collision_sensor` | int | Collision status (Check using `MaterialCollectingAgentParameters.SENSE_COLLIDED`) |
| `self.params.received_messages` | list[str] | List of messages from other agents within range |

### 2. Additional Object Information
| Detected Object (`sensor_object_type`) | Stored Value (`sensor_object_attribute`) |
| :--- | :--- |
| `SENSE_MATERIAL` (3) | **Radius** of the detected material |
| `SENSE_AGENT` (1) | **ID** of the detected agent |
| `SENSE_OBSTACLE` (2) | 0 (Fixed) |
| `SENSE_NONE` (0) | 0 (Fixed) |

### 3. Setting Actions (Write)
Select an action by assigning one of the following constants to `self.params.action`.

| Action Constant | Description | Required Additional Parameters |
| :--- | :--- | :--- |
| `ACT_GO_FORWARD` | Move forward | `self.params.velocity` (-MAX_VELOCITY to MAX_VELOCITY) |
| `ACT_ROTATE` | Rotate | `self.params.angular_velocity` (-MAX_ANGULAR_VELOCITY to MAX_ANGULAR_VELOCITY) |
| `ACT_STANDSTILL` | Stop | None |

### 4. Communication Settings (Write)
- `self.params.communication_message`: Assign a string to send it to nearby agents in the next step.

### Implementation Example (Inside the act method)
```python
def act(self):
    # 1. Check sensor information
    if self.params.collision_sensor == MaterialCollectingAgentParameters.SENSE_COLLIDED:
        # If collided, rotate to avoid
        self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
        self.params.angular_velocity = 45
    else:
        # Check for resource directly in front (center sensor is index 2)
        if self.params.sensor_object_type[2] == 3: 
            # If resource is found, move forward
            self.params.action = MaterialCollectingAgentParameters.ACT_GO_FORWARD
            self.params.velocity = MaterialCollectingAgentParameters.MAX_VELOCITY
        else:
            # Move randomly for exploration
            self.params.action = MaterialCollectingAgentParameters.ACT_ROTATE
            self.params.angular_velocity = random.uniform(-30, 30)

    # Update message
    self.params.communication_message = f"Agent {self.id} searching..."
```

## Running Experiments & Configuration
Start the simulation by running `main.py`.

### Key Parameters in main.py
Adjust these variables to make your experiments more efficient:
- `auto_executor`: Set to `True` to automatically repeat simulations.
- `double_speed_mode`: Set to `True` to increase rendering speed for faster experiments.

### Experiment Logs and CSV Data
After the simulation ends, experiment data is saved as a CSV file in the execution directory. You can analyze this data using Excel or Python (pandas) to evaluate algorithm performance.

- **Filename**: `log_YYYYMMDD_HHMMSS.csv`
- **Rows**:
    - `step`: Number of steps elapsed since the start.
- **Columns**:
    - [Total Resources Collected], [ID0 Collection], [ID1 Collection], ..., [ID5 Collection]

**Analysis Tips**:
- Plot the data with `step` on the x-axis and total resources on the y-axis.
- If a specific agent collects significantly more (or less) than others, check for bias in your exploration range or issues in the collision avoidance logic.

---

### Quick Start
1. Copy `my_material_collecting_agent.py` to create a new file.
2. Rewrite the `act` method to implement your own scanning and movement logic.
3. Edit `main.py` to register your agent in the simulator (see below).
4. Run `main.py`.

#### How to reflect your agent in main.py
To use your custom agent class (e.g., `OriginalAgent`), modify `main.py` as follows:

1. **Add Import**:
   Near the beginning of `main.py` (around line 7), import your class:
   ```python
   from original_agent_file import OriginalAgent  # Your filename and class name
   ```

2. **Replace Class Name**:
   Update the agent generation part within the `reset_environment` method of the `FrameManager` class (around line 41):
   ```python
   for i in range(num_agent):
       mc_agent = OriginalAgent()  # Change this to your class name
       mc_env.register_agent(mc_agent)
   ```