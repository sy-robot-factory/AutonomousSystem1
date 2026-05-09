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
git clone <リポジトリのURL>
cd AutonomousSystem1
```

### 2. シミュレータの起動
`uv` がインストールされている場合、以下のコマンドを実行するだけで、必要なライブラリ（pyglet, numpy等）のインストールが自動的に行われます。
```bash
uv sync
```
シミュレータを起動する際は以下のコマンドを実行します．
```bash
uv run main.py
```
※ `uv` を使用しない場合は、`pip install pyglet numpy` を実行した後に `python main.py` で起動してください。

## 実装方法
受講者の皆さんは、`agent.py` を継承した新しいエージェントクラスを作成してください。
実装のサンプルとして `my_material_collecting_agent.py` が用意されています。

### 実装のポイント
- **必須実装**: `act` メソッド内に、センサ情報に基づいた行動ロジックを記述してください。
- **内部変数**: 状態を保持するためのフィールド（変数）は、自身のサブクラス内に自由に追加して構いません。
- **禁止事項**: `agent.py` 本体や、その他のシステムクラス（シミュレータの基幹部分）は変更しないでください。

## 技術リファレンス

エージェントの行動決定は、`self.params` オブジェクトを読み書きすることで行います。

### 1. センサ情報の取得 (読み取り専用)
| 変数名 | 型 | 内容 |
| :--- | :--- | :--- |
| `self.params.sensor_object_type` | ndarray(5) | 5つのセグメントごとのオブジェクト種別 (0:なし, 1:障害物, 2:エージェント, 3:資源) |
| `self.params.sensor_object_distance` | ndarray(5) | 5つのセグメントごとのオブジェクトまでの距離 |
| `self.params.sensor_object_attribute` | ndarray(5) | オブジェクトの追加情報（資源の量など） |
| `self.params.collision_sensor` | int | 衝突状態 (`MaterialCollectingAgentParameters.SENSE_COLLIDED` で判定) |
| `self.params.received_messages` | list[str] | 通信範囲内にいる他エージェントからのメッセージリスト |

### 2. 行動の設定 (書き込み)
`self.params.action` に以下の定数を代入することで行動を選択します。

| アクション定数 | 説明 | 必要な追加パラメータ |
| :--- | :--- | :--- |
| `ACT_GO_FORWARD` | 前進 | `self.params.velocity` (0～MAX_VELOCITY) |
| `ACT_ROTATE` | 回転 | `self.params.angular_velocity` (-MAX～MAX_ANGULAR_VELOCITY) |
| `ACT_STANDSTILL` | 停止 | なし |

### 3. 通信の設定 (書き込み)
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
- `auto_executor`: 自動でシミュレーションを繰り返し実行する場合に使用します。
- `double_speed_mode`: `True` に設定すると、描画速度を上げて高速に実験を行えます。

### 実験記録とCSVファイルの見方
シミュレーション終了後、実行ディレクトリに実験データがCSVファイルとして保存されます。このデータをExcelやPythonのpandasなどで読み込むことで、アルゴリズムの性能を定量的に評価できます。

- **ファイル名**: `log_YYYYMMDD_HHMMSS.csv` (実行時の日時が反映されます)
- **列の構成**:
    - `step`: シミュレーション開始からの経過時間（ステップ数）。
    - `agent_0` ～ `agent_4`: 各エージェントがそのステップまでに獲得した資源の**累積数**。
    - `total`: チーム全体（5台合計）での資源獲得累積数。

**分析のヒント**:
- グラフの横軸を `step`、縦軸を `total` にしてプロットし、より多くの資源が獲得できる制御を目指しましょう。
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
