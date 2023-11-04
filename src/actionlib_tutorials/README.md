# action\_sample

## 想定
ロボットアームが目標に向けて動くようなロボットを想定しています。

## task\_server
HWインタフェースです。アームを持っている側の機器で動き、goalを受け取ることで動作します。

## task\_server
制御ノードインタフェースです。goalを送り、動作の様子をfeedbackなどで監視しています。

## 通信プロトコル
Task.actionで規定しているデータがやり取りできます。

## 参考
ロジックは公式wikiのものを大方流用しています 
http://wiki.ros.org/actionlib\_tutorials/Tutorials/SimpleActionClient
