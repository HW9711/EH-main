#!/usr/bin/env python3
"""
Diagram Renderer - 图表渲染脚本

功能：将DOT或JSON格式的图表描述渲染为PNG/SVG/PDF图片

参数：
    --input: 输入文件路径（.dot 或 .json）
    --output: 输出文件路径（支持 .png/.svg/.pdf）
    --format: 输出格式（png/svg/pdf，默认png）
    --engine: 布局引擎（dot/neato/fdp/sfdp，默认dot）
    --style: 样式模板路径（可选，JSON格式）
    --dpi: 分辨率（默认150，仅对PNG有效）

依赖：
    graphviz==0.20.1
"""

import argparse
import json
import sys
from pathlib import Path

try:
    from graphviz import Source
except ImportError:
    print("错误：未安装 graphviz 包，请执行: pip install graphviz==0.20.1")
    sys.exit(1)


def load_dot_file(filepath: str) -> str:
    """加载DOT文件内容"""
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"输入文件不存在: {filepath}")
    
    with open(path, 'r', encoding='utf-8') as f:
        return f.read()


def load_json_file(filepath: str) -> dict:
    """加载JSON文件内容"""
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"输入文件不存在: {filepath}")
    
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def json_to_dot(data: dict) -> str:
    """
    将JSON格式的图表描述转换为DOT语言
    
    JSON格式示例：
    {
        "type": "digraph",
        "name": "Flowchart",
        "attributes": {
            "rankdir": "TB"
        },
        "nodes": [
            {"id": "A", "label": "Start", "shape": "ellipse"},
            {"id": "B", "label": "Process", "shape": "box"}
        ],
        "edges": [
            {"from": "A", "to": "B", "label": "next"}
        ]
    }
    """
    graph_type = data.get("type", "digraph")
    graph_name = data.get("name", "G")
    attributes = data.get("attributes", {})
    nodes = data.get("nodes", [])
    edges = data.get("edges", [])
    
    lines = []
    
    # 图表头部
    lines.append(f"{graph_type} {graph_name} {{")
    
    # 图表属性
    if attributes:
        for key, value in attributes.items():
            lines.append(f'    {key}="{value}";')
    
    # 节点定义
    if nodes:
        lines.append("")
        lines.append("    // Nodes")
        for node in nodes:
            node_id = node.get("id")
            if not node_id:
                continue
            
            # 节点属性
            attrs = []
            if "label" in node:
                attrs.append(f'label="{node["label"]}"')
            if "shape" in node:
                attrs.append(f'shape={node["shape"]}')
            if "style" in node:
                attrs.append(f'style="{node["style"]}"')
            if "fillcolor" in node:
                attrs.append(f'fillcolor="{node["fillcolor"]}"')
            if "color" in node:
                attrs.append(f'color="{node["color"]}"')
            if "fontcolor" in node:
                attrs.append(f'fontcolor="{node["fontcolor"]}"')
            
            if attrs:
                lines.append(f'    {node_id} [{", ".join(attrs)}];')
            else:
                lines.append(f'    {node_id};')
    
    # 边定义
    if edges:
        lines.append("")
        lines.append("    // Edges")
        for edge in edges:
            from_node = edge.get("from")
            to_node = edge.get("to")
            
            if not from_node or not to_node:
                continue
            
            # 边属性
            attrs = []
            if "label" in edge:
                attrs.append(f'label="{edge["label"]}"')
            if "style" in edge:
                attrs.append(f'style="{edge["style"]}"')
            if "color" in edge:
                attrs.append(f'color="{edge["color"]}"')
            
            # 箭头类型
            arrow = "->" if graph_type == "digraph" else "--"
            
            if attrs:
                lines.append(f'    {from_node} {arrow} {to_node} [{", ".join(attrs)}];')
            else:
                lines.append(f'    {from_node} {arrow} {to_node};')
    
    lines.append("}")
    
    return "\n".join(lines)


def load_style_template(filepath: str) -> dict:
    """加载样式模板"""
    if not filepath:
        return {}
    
    path = Path(filepath)
    if not path.exists():
        print(f"警告：样式模板不存在: {filepath}，使用默认样式")
        return {}
    
    with open(path, 'r', encoding='utf-8') as f:
        return json.load(f)


def apply_style_to_dot(dot_content: str, style: dict) -> str:
    """
    将样式应用到DOT内容
    
    样式格式示例：
    {
        "node": {
            "shape": "box",
            "style": "rounded,filled",
            "fillcolor": "#E8F4F8"
        },
        "edge": {
            "color": "#333333"
        }
    }
    """
    if not style:
        return dot_content
    
    lines = dot_content.split('\n')
    new_lines = []
    in_graph = False
    
    for line in lines:
        stripped = line.strip()
        
        # 检测图表开始
        if stripped.startswith('digraph ') or stripped.startswith('graph '):
            in_graph = True
            new_lines.append(line)
            
            # 添加节点默认属性
            if 'node' in style:
                node_attrs = []
                for key, value in style['node'].items():
                    node_attrs.append(f'{key}="{value}"')
                if node_attrs:
                    new_lines.append(f'    node [{", ".join(node_attrs)}];')
            
            # 添加边默认属性
            if 'edge' in style:
                edge_attrs = []
                for key, value in style['edge'].items():
                    edge_attrs.append(f'{key}="{value}"')
                if edge_attrs:
                    new_lines.append(f'    edge [{", ".join(edge_attrs)}];')
            
            continue
        
        new_lines.append(line)
    
    return '\n'.join(new_lines)


def render_diagram(
    dot_content: str,
    output_path: str,
    format: str = 'png',
    engine: str = 'dot',
    dpi: int = 150
) -> str:
    """
    渲染图表为指定格式
    
    参数：
        dot_content: DOT语言内容
        output_path: 输出文件路径
        format: 输出格式（png/svg/pdf）
        engine: 布局引擎（dot/neato/fdp/sfdp）
        dpi: 分辨率（仅PNG有效）
    
    返回：
        生成的文件路径
    """
    # 创建Source对象
    src = Source(dot_content)
    
    # 设置引擎
    src.engine = engine
    
    # 设置格式
    src.format = format
    
    # 输出路径处理
    output = Path(output_path)
    output_dir = output.parent
    output_name = output.stem
    
    # 确保输出目录存在
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # 渲染
    try:
        rendered_path = src.render(
            filename=output_name,
            directory=str(output_dir),
            cleanup=True,
            format=format
        )
        
        # 重命名为用户指定的文件名
        final_path = output_dir / f"{output_name}.{format}"
        if Path(rendered_path) != final_path:
            Path(rendered_path).rename(final_path)
        
        return str(final_path)
    
    except Exception as e:
        raise RuntimeError(f"渲染失败: {str(e)}")


def main():
    parser = argparse.ArgumentParser(
        description='图表渲染脚本 - 将DOT/JSON格式的图表描述渲染为图片',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例：
  # 渲染DOT文件为PNG
  python render_diagram.py --input diagram.dot --output diagram.png
  
  # 渲染JSON文件为SVG
  python render_diagram.py --input diagram.json --output diagram.svg --format svg
  
  # 使用力导向布局
  python render_diagram.py --input network.dot --output network.png --engine fdp
  
  # 应用样式模板
  python render_diagram.py --input diagram.dot --output diagram.png --style modern.json
        """
    )
    
    parser.add_argument(
        '--input', '-i',
        required=True,
        help='输入文件路径（.dot 或 .json）'
    )
    
    parser.add_argument(
        '--output', '-o',
        required=True,
        help='输出文件路径（支持 .png/.svg/.pdf）'
    )
    
    parser.add_argument(
        '--format', '-f',
        choices=['png', 'svg', 'pdf'],
        default='png',
        help='输出格式（默认：png）'
    )
    
    parser.add_argument(
        '--engine', '-e',
        choices=['dot', 'neato', 'fdp', 'sfdp'],
        default='dot',
        help='布局引擎（默认：dot）'
    )
    
    parser.add_argument(
        '--style', '-s',
        help='样式模板路径（JSON格式）'
    )
    
    parser.add_argument(
        '--dpi', '-d',
        type=int,
        default=150,
        help='分辨率（默认：150，仅对PNG有效）'
    )
    
    args = parser.parse_args()
    
    try:
        # 判断输入格式
        input_ext = Path(args.input).suffix.lower()
        
        if input_ext == '.json':
            # 加载JSON并转换为DOT
            json_data = load_json_file(args.input)
            dot_content = json_to_dot(json_data)
        elif input_ext == '.dot':
            # 直接加载DOT
            dot_content = load_dot_file(args.input)
        else:
            raise ValueError(f"不支持的输入格式: {input_ext}，仅支持 .dot 和 .json")
        
        # 加载样式模板
        style = load_style_template(args.style) if args.style else {}
        
        # 应用样式
        dot_content = apply_style_to_dot(dot_content, style)
        
        # 渲染
        output_path = render_diagram(
            dot_content=dot_content,
            output_path=args.output,
            format=args.format,
            engine=args.engine,
            dpi=args.dpi
        )
        
        print(f"✓ 图表已生成: {output_path}")
        return 0
    
    except FileNotFoundError as e:
        print(f"错误: {e}", file=sys.stderr)
        return 1
    
    except ValueError as e:
        print(f"错误: {e}", file=sys.stderr)
        return 1
    
    except RuntimeError as e:
        print(f"渲染错误: {e}", file=sys.stderr)
        return 1
    
    except Exception as e:
        print(f"未知错误: {e}", file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
