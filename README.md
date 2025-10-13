# ChatGPTが作成した１００問

以下の100問で進める。前提: Ubuntu 22.04 + ROS 2 Humble on Docker、言語はPython、ros2_ws を想定。各問は「課題 → 達成条件」の順。

ワークスペース作成
達成: ~/ros2_ws/src 作成、colcon build が空ビルド成功

環境セットアップ
達成: source /opt/ros/humble/setup.bash と ros2_ws/install/setup.bash を .bashrc へ追記

最初のパッケージ
達成: rclpy 依存の minimal_talker_py 作成しビルド通過

最小ノード
達成: rclpy.init()→Node→spin→destroy_node() の骨組み実装

ロガー使用
達成: self.get_logger().info() で1回出力

タイマーで周期処理
達成: 0.5秒周期でカウント出力

Publisher作成
達成: /chatter に std_msgs/String を1Hzで送信

CLIで購読確認
達成: ros2 topic echo /chatter に送信内容が表示

Subscriber作成
達成: /chatter を受信してログ出力

Pub/Sub同時実行
達成: 同一パッケージに talker と listener 2本

コンソールスクリプト登録
達成: setup.py の entry_points 設定で ros2 run 起動可能

ament_python の使い方
達成: package.xml に <exec_depend>rclpy</exec_depend> 等を正しく記述

リマッピング
達成: ros2 run ... --ros-args -r /chatter:=/news で動作

ネームスペース
達成: --ros-args -r __ns:=/ns1 で /ns1/chatter となる

QoS: 信頼性
達成: ReliabilityPolicy.BEST_EFFORT に変更し動作比較

QoS: 履歴と深さ
達成: HistoryPolicy.KEEP_LAST 深さ=5で送受信

QoS: 耐久性
達成: DurabilityPolicy.TRANSIENT_LOCAL でLate-joiner受信を確認

コールバックグループ
達成: ReentrantCallbackGroup で重複処理がブロックされないことを観察

Executor: シングル vs マルチ
達成: MultiThreadedExecutor を適用し並行受信を確認

レート制御
達成: Clock と Rate で制御ループを実装

パラメータ宣言
達成: declare_parameter と get_parameter で整数パラメータ取得

動的パラメータ更新
達成: add_on_set_parameters_callback で検証ロジックを実装

YAMLからパラメータ読込
達成: --ros-args --params-file config.yaml で反映

パラメタイベント購読
達成: /parameter_events を購読し更新ログを表示

ノード間パラメータ取得
達成: rclpy.parameter_client 相当のクライアントで取得

サービスサーバ
達成: example_interfaces/srv/AddTwoInts 実装

サービスクライアント
達成: リクエスト送信し応答を受信

タイムアウト処理
達成: サービス呼び出しにタイムアウトを付与

アクションサーバ
達成: example_interfaces/action/Fibonacci をPythonで実装

アクションクライアント
達成: ゴール送信→フィードバック受信→結果取得

カスタムmsg作成
達成: my_msgs/msg/Num.msg を定義しPub/Sub

カスタムsrv作成
達成: my_msgs/srv/MultiplyTwoFloats.srv 定義し呼出

カスタムaction作成
達成: my_msgs/action/Countdown.action 定義し実装

インターフェイスのビルド依存
達成: rosidl_default_generators 等を設定し生成成功

メッセージ可視化
達成: ros2 interface show my_msgs/msg/Num で表示

文字列→メッセージ変換
達成: CLIの ros2 topic pub でカスタムmsg送信

型ミスマッチ検出
達成: 間違った型でビルドエラーが起きることを確認

シリアライズサイズ測定
達成: メッセージのサイズをログ出力（推定で可）

Python dataclass 連携
達成: dataclass ↔ msg の変換関数を作成

メッセージタイムスタンプ
達成: builtin_interfaces/msg/Time を埋め込み

ランチファイル基本
達成: launch で talker と listener を同時起動

ランチ引数
達成: DeclareLaunchArgument と LaunchConfiguration 使用

リマップとパラメータをlaunchで設定
達成: launchからトピック名とparamsを注入

条件分岐
達成: IfCondition でNode起動可否を制御

グループ化とNS
達成: GroupAction でネームスペース一括付与

TimerAction
達成: 起動後5秒で2本目ノード起動

ログレベル設定
達成: --ros-args --log-level debug をlaunchに含める

外部プロセス起動
達成: ExecuteProcess で ros2 topic echo 起動

LaunchTest
達成: launch_testing で起動確認テスト作成

IncludeLaunchDescription
達成: 子launchを取り込み分割構成にする

rqt_graph 代替確認
達成: CLIでノードとエッジの一覧を生成し保存

ros2 node list/info
達成: 出力をファイルに保存

ros2 topic list/info/type/bw
達成: 帯域計測を実施

ros2 service list/type/find
達成: 既存サービスの型を特定

ros2 action list/type/info
達成: 既存アクションを列挙

ros2 doctor
達成: 診断結果を確認し警告ゼロ

ros2 run --prefix
達成: gdb -ex run --args でノードを起動

ros2 bag record
達成: /chatter を記録し再生

/clock とシムタイム
達成: use_sim_time 有効でbag再生に同期

Bagのフィルタ再生
達成: --regex や --topics で再生対象を限定

例外処理
達成: サブスクのコールバックで例外を握り潰さずログ+継続

終了シグナル対応
達成: signal でCtrl-C受信時のクリーンアップ

単体テスト(pytest)
達成: Publisherの出力をモックで検証

ament_lint 導入
達成: ament_flake8 と ament_pep257 を通過

型ヒント
達成: 全関数に型注釈を付与

CI用shellスクリプト
達成: ビルド→テスト→lintを一括実行

ログフォーマット統一
達成: 一貫したprefixと構造化ログ

再接続ロジック
達成: 一時的なDDS断後も自動復帰

スロットリング
達成: 高頻度入力をデバウンス

バックプレッシャ
達成: 受信処理が遅い時にドロップ戦略を実装

複数ノード同一プロセス
達成: 2ノードを1プロセスで作成しExecutorで管理

コンポーザブルノード
達成: Pythonの ComposableNodeContainer 起動

ライフサイクルノード
達成: unconfigured→inactive→active 遷移を実装

ライフサイクル管理
達成: ros2 lifecycle CLIで遷移操作

トピック統計
達成: topic_statistics を有効化しレイテンシ取得

時刻・タイムソース
達成: 系統時刻とシミュレーション時刻の違いをログ

TF2ブロードキャスト
達成: base_link→laser の静的TF送信

TF2リスナ
達成: 任意時刻の変換取得

2Dスキャン処理
達成: sensor_msgs/LaserScan を受け簡易レンジ統計出力

可視化CSV出力
達成: 任意トピックをCSVに落として外部で可視化

ネームドパラメータ・プロファイル
達成: launchのparameters=[{...}]とYAML併用

DDSミドルウェア切替
達成: RMW_IMPLEMENTATION を設定しFastDDS↔CycloneDDS比較（可用なら）

マルチマシン準備
達成: Dockerブリッジとホスト間でトピック疎通を確認

発見設定
達成: Fast DDS の環境変数で発見挙動を調整（範囲限定）

セキュリティ概観
達成: SROS2でキーストア作成と有効化の手順確認

リマップファイル
達成: YAMLのリマップ表をlaunchから適用

階層NS設計
達成: /robot1/sensors/laser のような階層で整理

トピック命名規約
達成: 名称・型・周期をREADMEに文書化

設定分離
達成: dev/prod用パラメータファイルを分割

リリース手順書
達成: バージョンタグ、CHANGELOG、依存固定を整備

シリアルブリッジ
達成: 擬似デバイスで行単位を std_msgs/String に流すノード

コマンド速度I/O
達成: /cmd_vel を受けて簡易運動学計算結果を出力

画像メッセージ基礎
達成: sensor_msgs/Image を擬似生成してPub

サービスで設定反映
達成: サービス呼出で内部状態を切替

アクションで長時間処理
達成: 進捗フィードバックを適切に送出

例外設計指針
達成: 全入出力経路で例外→リトライ→フェイルセーフ

パフォーマンステスト
達成: 1kHz相当の空メッセージPubのCPU率測定

メモリ使用量確認
達成: /proc 参照やtracemallocで増分確認

プロファイリング
達成: cProfile でホットスポットを抽出

完成デモ
達成: launch一発でPub/Sub/Service/Action/TF/bag記録が起動し、READMEの手順で再現可能
