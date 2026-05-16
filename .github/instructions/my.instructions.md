---
description: "每次准备发送最终回复前，必须调用提问工具获取下一步指示；用于防止任务结束后直接收尾。"
applyTo: "**"
---

# 结束前提问规则

- 当前请求完成、准备总结、准备结束对话、没有明确下一步可执行内容时，必须在最终回复前调用提问工具。
- 本轮早些时候调用过提问工具不算；只要后续又执行了工具、分析、编辑或生成了新内容，最终回复前仍需再次调用。
- 优先调用 `vscode_askQuestions`；若环境中显示为 `askQuestions`，则调用 `askQuestions`。
- 若用户明确选择“不需要，结束”，再给出简短最终总结。

## 最终回复前机械自检

- 如果下一条消息将发送到 `final` 通道，先停止生成正文。
- 检查本次准备发送 `final` 前，最近一次动作是否已经调用 `vscode_askQuestions` 或 `askQuestions`。
- 若最近一次动作不是提问工具调用，必须先调用提问工具，再根据用户选择继续或总结。
- 若提问工具返回后又进行了任何工具调用、文件修改、终端命令、分析扩展或新增总结内容，再次进入 `final` 前必须重新调用提问工具。
- 不要把“之前已经问过一次”“用户应该没有下一步”“只是解释原因”当作跳过提问的理由。
- 只有当用户在最近一次提问工具中明确选择“不需要，结束”或等价表述时，才允许直接发送简短最终回复。

# Echo 自动接入规则

- 每轮开始处理任务前，优先读取当前工作区规则文件 `.github/copilot-instructions.md` 与本文件。
- 若存在 `D:/KnowledgeBase/MarinaEcho/EchoWorkspace/AGENTS.md`，同时读取该文件作为 Echo 全局行为背景。
- 若存在 `D:/KnowledgeBase/MarinaEcho/EchoWorkspace/echo_memory/`，按需读取其中的 `project_context.md`、`active_status.md`、`lessons_learned.md`、`archived_memory.md`、`system_redlines.md`、`server_ops.md`、`health_baseline.md`、`user_preferences.md`。
- Echo 工作区只作为长期记忆和偏好来源；除非用户明确要求“接入 EchoWorkspace”或给出 EchoWorkspace 路径，当前 VS Code 工作区始终是默认分析与修改对象。
- 不要因为读取 Echo 记忆而把当前任务误判为 EchoWorkspace 项目任务。
- 未经用户明确要求，不要修改 EchoWorkspace 或 echo_memory 文件。
