# 文档

N32H473CE 三相 FOC 固件。只有三份要读的文档，按"你是谁"选一份即可（本页是索引）：

| 文档 | 给谁 | 内容 |
|---|---|---|
| [USER.md](USER.md) | 使用者 / 上位机开发者 | 连接、读写参数、保存、标定、备份、OTA、故障排查、FAQ |
| [DEV.md](DEV.md) | 改固件的人 | 架构、endpoint 协议、`params.json` 与代码生成、配置持久化、Flash/OTA 契约、CAN 移植、测试与发布 |
| [parameters.md](parameters.md) | 所有人 | 参数手册（由 `params.json` 自动生成，勿手改） |

构建命令、仓库结构见根目录 [README.md](../README.md)；给 AI 的短契约见
[AGENTS.md](../AGENTS.md)。

## 版本

| 项目 | 当前值 | 设备上怎么读 |
|---|---|---|
| 固件 | `2.00` | `system.version` |
| endpoint 协议 | `2` | endpoint 0 的版本探测（偏移 `0xFFFFFFFF`） |
| 配置记录 schema | `1` | `system.config_schema` |
| 配置块序号 | 每次保存 +1 | `system.prefs_sequence` |

描述符的 CRC 就是接口版本，每次增删参数都会变（具体值用 endpoint 0 探测，或
`mc_ep.py descriptor` 看）。CRC 变了以后旧上位机的请求会被 `EP_ERR_VERSION`
拒绝，不会出现"名字对上了、类型对不上"的静默错误。

## 五分钟上手

```bash
pip install pyserial
python tools/mc_ep.py --port COM7 tree            # 参数树
python tools/mc_ep.py --port COM7 get config.i_max
python tools/mc_ep.py --port COM7 set config.i_max 3.5
python tools/mc_ep.py --port COM7 call action.save
python tools/mc_ep.py --port COM7 get system.save_status    # 1 = 保存成功
```

改完参数要 `action.save` 才写 flash；门极使能时保存会被拒绝（先
`call action.idle_mode`）。细节见 [USER.md](USER.md)。

## 一页纸数据流

```
上位机 ──USB CDC──> endpoint_serial.c ──> endpoint_handler(id, op, ...)  [生成]
                                              └─ 直接读写 params.json 里的变量
主循环 ──> run_fsm_slow() ──> save_preferences() ──> TLV + CRC32 ──> 双页 ping-pong
控制 ISR (40 kHz) ──> 采样 → 故障检查 → 状态机 → CAN
```

参数表、描述符、持久化表、参数手册全部由根目录 `params.json` 生成：

```bash
make endpoints   # 重新生成
make check       # 校验生成物是否最新（提交前跑）
```
