# Universal Robots ROS Driver と Client Library の制御モードに関するメモ

このドキュメントは、Universal Robots ROS Driver と Universal Robots Client Library の間で、特にトルク制御、位置制御、速度制御における制御モードの整合性について議論した内容をまとめたものです。

## 1. Universal Robots Client Library の制御モード

`Universal_Robots_Client_Library`は、`include/ur_client_library/comm/control_mode.h`で以下の主要な制御モードを定義しています。

*   `MODE_SERVOJ`: 関節空間での位置制御 (servoj)
*   `MODE_SPEEDJ`: 関節空間での速度制御 (speedj)
*   `MODE_SPEEDL`: タスク空間での速度制御 (speedl)
*   `MODE_POSE`: タスク空間での位置制御 (pose)
*   `MODE_TORQUE`: トルク制御
*   `MODE_PD_CONTROLLER_JOINT`: 関節空間でのPD制御
*   `MODE_PD_CONTROLLER_TASK`: タスク空間でのPD制御

`ur_driver.h`の`writeJointCommand`関数は、これらの`comm::ControlMode`を引数として受け取り、ロボットにコマンドを送信します。

## 2. 制御モードの比較とロジックの場所

| 特徴             | `MODE_TORQUE`                               | `MODE_PD_CONTROLLER_JOINT`                  | `MODE_PD_CONTROLLER_TASK`                   |
| :--------------- | :------------------------------------------ | :------------------------------------------ | :------------------------------------------ |
| **制御対象**     | 各関節に直接印加するトルク                  | 関節空間での目標位置                        | タスク空間（Cartesian空間）での目標位置     |
| **制御ロジックの場所** | **外部システム** (例: ROSドライバ、PC上のカスタムコントローラ)がPD制御などの高レベルな制御ロジックを持ち、計算されたトルク値をロボットに送信する。 | **ロボットコントローラ内部**でPD制御ループが実行される。 | **ロボットコントローラ内部**でPD制御ループが実行される。 |
| **ロボットへの入力** | 各関節の目標トルク値 (6軸分)                | 各関節の目標位置 (6軸分) とPDゲイン         | 目標Cartesianポーズ (x,y,z,rx,ry,rz) とPDゲイン |
| **ロボットの動作** | 受け取ったトルク値を直接関節に印加する。    | 目標位置と現在の位置の差に基づいて、ロボット自身がトルクを計算し、関節を制御する。 | 目標Cartesianポーズと現在のポーズの差に基づいて、ロボット自身がトルクを計算し、関節を制御する。 |
| **主な用途**     | 外部で複雑な制御アルゴリズム（例: インピーダンス制御、力制御）を実装し、その結果として得られるトルクをロボットに直接与えたい場合。 | ロボットに特定の関節位置を追従させたいが、その追従ロジックをロボット内部に任せたい場合。 | ロボットに特定のCartesianポーズを追従させたいが、その追従ロジックをロボット内部に任せたい場合。 |

## 3. ROS Driver の現在の実装と Client Library との整合性

### トルク制御

*   **ROS Driver の実装**: ROS Driver のトルク制御は、`ros_control`の`effort_controllers/JointGroupEffortController`を使用し、`torque_control.urscript`を介してRTDE経由でロボットに直接トルク値を送信します。
*   **整合性**: この実装は、`Universal_Robots_Client_Library`の`MODE_TORQUE`モードと完全に一致しています。ROS側で計算されたトルク値がロボットに直接与えられ、ロボットはその値を実行します。

### 位置制御・速度制御

*   **ROS Driver の実装**: ROS Driver の標準的な位置制御（例: `scaled_pos_joint_traj_controller`）や速度制御（例: `joint_group_vel_controller`）は、ROS側（`ros_control`のコントローラ）でPD/PID制御などのロジックを持っています。これらのコントローラは目標位置や目標速度を計算し、それをロボットに送信します。
*   **整合性**: この場合、`Universal_Robots_Client_Library`は、ROSコントローラから受け取った目標位置や目標速度を、ロボットが解釈できる形式（例: `MODE_SERVOJ`や`MODE_SPEEDJ`）に変換してロボットに送る役割を担っています。ロボット内部にも低レベルなサーボ制御ループはありますが、ROS側でより高レベルな軌道追従や速度制御のロジックが動いています。

### 結論

*   ROS Driver の現在のトルク制御実装は、`Universal_Robots_Client_Library`の`MODE_TORQUE`モードと整合性が取れています。
*   ROS Driver の標準的な位置制御・速度制御実装は、ROS側で高レベルな制御ロジックを持ち、Client Library の対応する制御モード（`MODE_SERVOJ`, `MODE_SPEEDJ`など）を介してロボットにコマンドを送信しています。

もし、`Universal_Robots_Client_Library`が提供する`MODE_PD_CONTROLLER_JOINT`や`MODE_PD_CONTROLLER_TASK`といったロボット内部でのPD制御機能を利用したい場合は、ROS Driver 側に新しいROSコントローラを実装し、そのコントローラが Client Library の該当する制御モードを呼び出すように変更する必要があります。
