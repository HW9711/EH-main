# DOT语言语法规范

## 目录

1. [概述](#概述)
2. [基础语法](#基础语法)
3. [图表类型](#图表类型)
4. [节点定义](#节点定义)
5. [边定义](#边定义)
6. [属性设置](#属性设置)
7. [子图与聚类](#子图与聚类)
8. [常用模板](#常用模板)
9. [最佳实践](#最佳实践)

---

## 概述

DOT是Graphviz工具使用的图形描述语言，用于定义节点、边和图形属性。本规范提供DOT语言的核心语法和常用模式。

**核心特点：**
- 声明式语言：描述图形结构
- 自动布局：由布局引擎决定最终位置
- 丰富的属性：支持样式、颜色、字体等自定义

---

## 基础语法

### 图表结构

```dot
graph G {
    // 无向图
    A -- B;
}

digraph G {
    // 有向图
    A -> B;
}
```

### 注释

```dot
// 单行注释

/*
 * 多行注释
 */
```

### 标识符规则

- 字母、数字、下划线
- 以字母开头
- 大小写敏感
- 包含特殊字符时用引号：`"节点A"`

---

## 图表类型

### 无向图 (graph)

```dot
graph Network {
    A -- B;
    B -- C;
    C -- A;
}
```

**适用场景：**
- 关系网络
- 无向连接
- 对称关系

### 有向图 (digraph)

```dot
digraph Flowchart {
    A -> B;
    B -> C;
}
```

**适用场景：**
- 流程图
- 数据流
- 层级关系

---

## 节点定义

### 基本定义

```dot
digraph G {
    A;                  // 最简单定义
    B [label="Node B"]; // 带标签
    "节点C";            // 引号包裹（支持中文）
}
```

### 节点形状

```dot
digraph Shapes {
    // 基本形状
    box_node [shape=box, label="矩形"];
    ellipse_node [shape=ellipse, label="椭圆"];
    circle_node [shape=circle, label="圆"];
    diamond_node [shape=diamond, label="菱形"];
    
    // 特殊形状
    point_node [shape=point];           // 点
    plaintext_node [shape=plaintext];   // 纯文本
    polygon_node [shape=polygon, sides=6];  // 多边形
    
    // 流程图标准形状
    start [shape=ellipse, label="开始"];
    process [shape=box, label="处理"];
    decision [shape=diamond, label="判断"];
    io [shape=parallelogram, label="输入/输出"];
    end [shape=ellipse, label="结束"];
}
```

**常用形状列表：**

| 形状 | 名称 | 适用场景 |
|------|------|---------|
| `box` | 矩形 | 处理步骤 |
| `ellipse` | 椭圆 | 开始/结束 |
| `circle` | 圆形 | 连接点 |
| `diamond` | 菱形 | 决策判断 |
| `parallelogram` | 平行四边形 | 输入/输出 |
| `hexagon` | 六边形 | 准备/初始化 |
| `octagon` | 八边形 | 手动操作 |
| `doublecircle` | 双圆 | 终止节点 |
| `record` | 记录 | 结构化数据 |
| `plaintext` | 纯文本 | 无边框标签 |

### 节点样式

```dot
digraph Styles {
    // 填充
    filled_node [style=filled, fillcolor="#E8F4F8"];
    
    // 圆角
    rounded_node [style="rounded,filled", fillcolor="#FFE4B5"];
    
    // 虚线边框
    dashed_node [style=dashed];
    
    // 加粗
    bold_node [style=bold];
    
    // 组合样式
    complex_node [style="rounded,filled,bold", 
                  fillcolor="#90EE90", 
                  color="#2E8B57"];
}
```

---

## 边定义

### 基本定义

```dot
digraph G {
    A -> B;                    // 基本边
    A -> B [label="连接"];      // 带标签
    A -> B [label="是", fontcolor=green];  // 彩色标签
}
```

### 边样式

```dot
digraph Edges {
    // 线型
    solid_edge [style=solid];
    dashed_edge [style=dashed];
    dotted_edge [style=dotted];
    bold_edge [style=bold];
    
    // 箭头
    normal_arrow [arrowhead=normal];
    vee_arrow [arrowhead=vee];
    dot_arrow [arrowhead=dot];
    none_arrow [arrowhead=none];  // 无箭头
    
    // 双向箭头
    bidirection [dir=both];
    
    // 无箭头方向
    no_dir [dir=none];
}
```

**常用箭头类型：**

| 箭头 | 名称 | 适用场景 |
|------|------|---------|
| `normal` | 标准箭头 | 默认 |
| `vee` | V形箭头 | 强调方向 |
| `dot` | 圆点 | 特殊标记 |
| `box` | 方框 | 目标端点 |
| `diamond` | 菱形 | 特殊连接 |
| `none` | 无箭头 | 双向关系 |

### 多边连接

```dot
digraph Multiple {
    A -> B;
    A -> C;
    A -> D;
    
    // 同一对节点多个边
    E -> F [label="路径1"];
    E -> F [label="路径2", style=dashed];
}
```

---

## 属性设置

### 图形属性

```dot
digraph GraphAttrs {
    // 布局方向
    rankdir=TB;  // TB(上到下)、LR(左到右)、BT(下到上)、RL(右到左)
    
    // 节点间距
    nodesep=0.5;    // 同层节点间距
    ranksep=1.0;    // 层级间距
    
    // 图形大小
    size="8,11";    // 最大尺寸（英寸）
    ratio=fill;     // 填充模式
    
    // 字体
    fontname="Arial";
    fontsize=14;
    
    // 背景
    bgcolor="white";
    
    // 节点和边的默认属性
    node [fontname="Arial", fontsize=12];
    edge [fontname="Arial", fontsize=10];
}
```

### 全局默认属性

```dot
digraph Defaults {
    // 默认节点属性
    node [shape=box, style="rounded,filled", fillcolor="#E8F4F8"];
    
    // 默认边属性
    edge [color="#333333", arrowsize=0.8];
    
    // 之后定义的节点和边会继承这些默认值
    A -> B;
    B -> C;
}
```

---

## 子图与聚类

### 子图 (subgraph)

```dot
digraph Subgraphs {
    // 子图用于逻辑分组
    subgraph cluster_0 {
        label="子系统A";
        A -> B;
        B -> C;
    }
    
    subgraph cluster_1 {
        label="子系统B";
        D -> E;
    }
    
    // 子图间连接
    C -> D;
}
```

### 聚类样式

```dot
digraph ClusterStyle {
    subgraph cluster_backend {
        label="后端服务";
        style="rounded,filled";
        fillcolor="#E8F4F8";
        color="#4A90E2";
        fontcolor="#4A90E2";
        
        api [label="API服务"];
        db [label="数据库"];
    }
    
    subgraph cluster_frontend {
        label="前端应用";
        style="rounded,filled";
        fillcolor="#FFF4E6";
        color="#F5A623";
        
        web [label="Web应用"];
        mobile [label="移动端"];
    }
    
    web -> api;
    mobile -> api;
    api -> db;
}
```

---

## 常用模板

### 流程图模板

```dot
digraph Flowchart {
    // 全局设置
    rankdir=TB;
    node [shape=box, style="rounded,filled", fillcolor="#E8F4F8"];
    edge [color="#333333"];
    
    // 特殊节点
    Start [shape=ellipse, fillcolor="#90EE90", label="开始"];
    End [shape=ellipse, fillcolor="#FFB6C1", label="结束"];
    Decision [shape=diamond, fillcolor="#FFE4B5", label="条件判断?"];
    
    // 流程
    Start -> "步骤1";
    "步骤1" -> Decision;
    Decision -> "分支A" [label="是"];
    Decision -> "分支B" [label="否"];
    "分支A" -> End;
    "分支B" -> End;
}
```

### 组织架构图模板

```dot
digraph OrgChart {
    // 全局设置
    rankdir=TB;
    node [shape=box, style="rounded,filled", fillcolor="#E8F4F8"];
    edge [color="#666666"];
    
    // 层级结构
    CEO [fillcolor="#4A90E2", fontcolor=white];
    
    {rank=same; CTO; CFO;}
    
    CEO -> CTO;
    CEO -> CFO;
    
    CTO -> "技术部";
    CTO -> "产品部";
    CFO -> "财务部";
    CFO -> "法务部";
}
```

### 系统架构图模板

```dot
digraph SystemArchitecture {
    // 全局设置
    rankdir=TB;
    node [shape=box, style="rounded,filled"];
    edge [color="#333333"];
    
    // 分层
    subgraph cluster_presentation {
        label="表现层";
        style=dashed;
        color="#4A90E2";
        
        web [label="Web应用", fillcolor="#E8F4F8"];
        mobile [label="移动应用", fillcolor="#E8F4F8"];
    }
    
    subgraph cluster_business {
        label="业务层";
        style=dashed;
        color="#F5A623";
        
        api [label="API网关", fillcolor="#FFF4E6"];
        user_service [label="用户服务", fillcolor="#FFF4E6"];
        order_service [label="订单服务", fillcolor="#FFF4E6"];
    }
    
    subgraph cluster_data {
        label="数据层";
        style=dashed;
        color="#50C878";
        
        mysql [label="MySQL", fillcolor="#E8F8E8"];
        redis [label="Redis", fillcolor="#E8F8E8"];
    }
    
    // 连接
    web -> api;
    mobile -> api;
    api -> user_service;
    api -> order_service;
    user_service -> mysql;
    user_service -> redis;
    order_service -> mysql;
}
```

### 控制框图模板

```dot
digraph ControlSystem {
    rankdir=LR;
    node [shape=box, style="rounded,filled", fillcolor="#E8F4F8"];
    edge [color="#333333", arrowsize=0.8];
    
    // 控制系统元素
    Input [shape=parallelogram, label="输入"];
    Controller [label="控制器"];
    Plant [label="被控对象"];
    Sensor [label="传感器"];
    Output [shape=parallelogram, label="输出"];
    
    // 主回路
    Input -> Controller;
    Controller -> Plant;
    Plant -> Output;
    
    // 反馈回路
    Plant -> Sensor [style=dashed];
    Sensor -> Controller [label="反馈", style=dashed];
}
```

---

## 最佳实践

### 1. 命名规范

```dot
// 推荐：使用有意义的标识符
start_process [label="开始处理"];
validate_input [label="验证输入"];
send_notification [label="发送通知"];

// 避免：无意义命名
node1 [label="开始"];
node2 [label="验证"];
```

### 2. 结构化组织

```dot
digraph WellStructured {
    // ===== 全局设置 =====
    rankdir=TB;
    node [shape=box];
    
    // ===== 节点定义区 =====
    Start [shape=ellipse, label="开始"];
    Process [label="处理"];
    End [shape=ellipse, label="结束"];
    
    // ===== 边定义区 =====
    Start -> Process;
    Process -> End;
}
```

### 3. 注释说明

```dot
digraph CommentedGraph {
    // 设置布局方向为从上到下
    rankdir=TB;
    
    // 定义主要节点
    A [label="步骤A"];
    B [label="步骤B"];
    
    // 定义主要流程
    A -> B;  // A完成后执行B
}
```

### 4. 样式一致性

```dot
digraph ConsistentStyle {
    // 统一使用相同颜色方案
    node [fillcolor="#E8F4F8", style="rounded,filled"];
    
    // 同类节点使用相同样式
    process1 [label="处理1"];
    process2 [label="处理2"];
    process3 [label="处理3"];
    
    // 特殊节点显式覆盖
    special [fillcolor="#FFE4B5", label="特殊处理"];
}
```

### 5. 避免常见错误

```dot
// 错误：中文标识符未加引号
节点A -> 节点B;  // ❌

// 正确：中文标识符加引号
"节点A" -> "节点B";  // ✅

// 错误：属性值未加引号
node [label=处理步骤];  // ❌

// 正确：属性值加引号
node [label="处理步骤"];  // ✅
```

---

## 属性快速参考

### 节点属性

| 属性 | 说明 | 示例 |
|------|------|------|
| `label` | 显示文本 | `label="步骤1"` |
| `shape` | 形状 | `shape=box` |
| `style` | 样式 | `style="rounded,filled"` |
| `fillcolor` | 填充颜色 | `fillcolor="#E8F4F8"` |
| `color` | 边框颜色 | `color="#4A90E2"` |
| `fontname` | 字体 | `fontname="Arial"` |
| `fontsize` | 字号 | `fontsize=12` |
| `fontcolor` | 字体颜色 | `fontcolor="#333333"` |
| `width` | 最小宽度 | `width=2` |
| `height` | 最小高度 | `height=1` |

### 边属性

| 属性 | 说明 | 示例 |
|------|------|------|
| `label` | 边标签 | `label="是"` |
| `color` | 颜色 | `color="#4A90E2"` |
| `style` | 线型 | `style=dashed` |
| `arrowhead` | 箭头类型 | `arrowhead=vee` |
| `arrowsize` | 箭头大小 | `arrowsize=0.8` |
| `dir` | 方向 | `dir=both` |
| `penwidth` | 线宽 | `penwidth=2` |

### 图形属性

| 属性 | 说明 | 示例 |
|------|------|------|
| `rankdir` | 布局方向 | `rankdir=TB` |
| `nodesep` | 节点间距 | `nodesep=0.5` |
| `ranksep` | 层级间距 | `ranksep=1.0` |
| `size` | 图形大小 | `size="8,11"` |
| `dpi` | 分辨率 | `dpi=150` |
| `bgcolor` | 背景色 | `bgcolor="white"` |

---

## 总结

DOT语言核心要点：

1. **声明式思维**：描述"有什么"，而非"怎么画"
2. **属性继承**：合理使用默认属性减少重复
3. **结构清晰**：良好的组织和注释便于维护
4. **样式一致**：统一的视觉风格提升可读性
5. **工具辅助**：善用布局引擎的自动排版能力
