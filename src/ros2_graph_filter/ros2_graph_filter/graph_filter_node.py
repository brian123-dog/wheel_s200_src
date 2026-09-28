#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ros2_graph_filter 核心节点 —— graph_filter_node

功能说明：
  1. 创建一个临时 ROS2 节点，通过 rclpy 原生 API 获取当前运行图中的
     所有节点名称、话题名称、以及发布者/订阅者的对应关系。
  2. 读取 YAML 配置文件，根据 hide_nodes / hide_topics 正则规则
     过滤掉不需要展示的节点和话题。
  3. 根据 aggregate_groups 配置，将指定的多个节点聚合为一个逻辑组节点，
     隐藏组内连线，将组外连接统一关联到聚合组节点。
  4. 生成 Graphviz DOT 格式的拓扑描述文件，并调用 graphviz 库渲染为 SVG。

使用方式：
  ros2 run ros2_graph_filter ros2_graph_filter [配置文件路径] [输出SVG路径]
  ros2 launch ros2_graph_filter graph_filter.launch.py

依赖：
  - rclpy (ROS2 Python 客户端库)
  - graphviz (Python graphviz 绑定)
  - PyYAML (YAML 解析)
"""

import argparse
import os
import re
import sys
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
import yaml

# ---------------------------------------------------------------------------
# 尝试导入 graphviz；若未安装则给出友好提示
# ---------------------------------------------------------------------------
try:
    import graphviz
    HAS_GRAPHVIZ = True
except ImportError:
    HAS_GRAPHVIZ = False
    print("[ERROR] 未安装 graphviz Python 包，请执行:")
    print("        pip install graphviz")
    print("        或 sudo apt install python3-graphviz")
    sys.exit(1)


# ===========================================================================
# 常量定义 —— 黑白灰度样式，适合学术论文印刷
# ===========================================================================

# 聚合组虚拟节点前缀：用于区分真实节点和聚合后的虚拟节点
AGGREGATE_PREFIX = "__aggregate__:"

# 普通节点样式：白底黑框圆角矩形，适合黑白印刷
STYLE_NODE = {
    'shape': 'box',
    'style': 'rounded,filled',
    'fillcolor': 'white',
    'color': 'black',
    'fontcolor': 'black',
    'fontsize': '11',
    'fontname': 'DejaVu Sans',
}

# 话题节点样式：浅灰椭圆，在黑白图中与白色节点形成层次区分
STYLE_TOPIC = {
    'shape': 'ellipse',
    'style': 'filled',
    'fillcolor': '#F0F0F0',
    'color': 'black',
    'fontcolor': 'black',
    'fontsize': '10',
    'fontname': 'DejaVu Sans',
}

# 聚合组节点样式：白底虚线粗边框圆角矩形，论文中虚线框依然有效
STYLE_AGGREGATE = {
    'shape': 'box',
    'style': 'rounded,filled,dashed',
    'fillcolor': 'white',
    'color': 'black',
    'fontcolor': 'black',
    'fontsize': '13',
    'fontname': 'DejaVu Sans',
    'penwidth': '1.5',
}

# 连线样式
STYLE_EDGE = {
    'arrowhead': 'normal',
    'color': 'black',
    'fontcolor': 'black',
    'fontsize': '9',
    'fontname': 'DejaVu Sans',
}


# ===========================================================================
# 工具函数
# ===========================================================================

def matches_any_pattern(name: str, patterns: List[str]) -> bool:
    """
    检查名称是否匹配给定列表中的任一正则表达式。

    Args:
        name:     待检查的节点或话题全名（含命名空间）
        patterns: 正则表达式字符串列表

    Returns:
        True 如果匹配任一正则，否则 False
    """
    for pattern in patterns:
        try:
            if re.search(pattern, name):
                return True
        except re.error as e:
            print(f"[WARN] 正则表达式 '{pattern}' 无效: {e}")
    return False


def get_canonical_topic_name(topic_name: str) -> str:
    """
    规范化话题名称：确保以 '/' 开头。

    Args:
        topic_name: 原始话题名称

    Returns:
        以 '/' 开头的规范化话题名称
    """
    if not topic_name.startswith('/'):
        return '/' + topic_name
    return topic_name


def get_canonical_node_name(name: str, namespace: str) -> str:
    """
    生成节点的完整限定名（fully qualified name）。

    Args:
        name:      节点短名称（如 "rm_driver"）
        namespace: 节点命名空间（如 "/"）

    Returns:
        完整限定名（如 "/rm_driver"）
    """
    ns = namespace.rstrip('/') if namespace else ''
    if ns and not ns.startswith('/'):
        ns = '/' + ns
    if ns == '/':
        return '/' + name
    return ns + '/' + name


# ===========================================================================
# 配置结构
# ===========================================================================

class FilterConfig:
    """
    过滤配置的数据结构，封装 YAML 配置文件中的所有规则。

    Attributes:
        hide_nodes:       需要隐藏的节点正则表达式列表
        hide_topics:      需要隐藏的话题正则表达式列表
        aggregate_groups: 聚合组定义列表，每个元素为 dict:
                          {name, nodes, style}
    """

    def __init__(self, config_dict: dict):
        self.hide_nodes: List[str] = config_dict.get('hide_nodes', [])
        self.hide_topics: List[str] = config_dict.get('hide_topics', [])
        self.aggregate_groups: List[dict] = config_dict.get('aggregate_groups', [])

    @classmethod
    def from_yaml(cls, yaml_path: str) -> 'FilterConfig':
        """
        从 YAML 文件加载配置。

        Args:
            yaml_path: YAML 文件路径

        Returns:
            FilterConfig 实例
        """
        if not os.path.exists(yaml_path):
            print(f"[ERROR] 配置文件不存在: {yaml_path}")
            sys.exit(1)

        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        if data is None:
            print(f"[WARN] 配置文件 {yaml_path} 为空，使用空规则")
            data = {}

        return cls(data)


# ===========================================================================
# 图拓扑采集器
# ===========================================================================

class GraphTopologyCollector:
    """
    通过 ROS2 原生 API 采集当前运行图的拓扑信息。

    内部维护：
      - 所有节点名称集合
      - 所有话题名称集合
      - 话题 -> 发布者节点列表 的映射
      - 话题 -> 订阅者节点列表 的映射
    """

    def __init__(self, node: Node):
        """
        Args:
            node: 一个已初始化的 rclpy Node 实例
        """
        self._node = node
        self.node_names: Set[str] = set()
        self.topic_names: Set[str] = set()
        self.publishers_map: Dict[str, List[str]] = {}   # topic -> [node1, node2, ...]
        self.subscribers_map: Dict[str, List[str]] = {}  # topic -> [node1, node2, ...]

    def collect(self):
        """
        执行拓扑信息采集。

        步骤：
          1. 获取所有节点名和命名空间，构建完整限定名
          2. 获取所有话题名和类型
          3. 对每个话题，获取其发布者和订阅者节点列表
        """
        print("[INFO] 正在获取 ROS2 图拓扑信息...")

        # ---- 步骤 1: 获取所有节点 ----
        node_info_list = self._node.get_node_names_and_namespaces()
        for name, namespace in node_info_list:
            full_name = get_canonical_node_name(name, namespace)
            self.node_names.add(full_name)
        print(f"[INFO]   发现 {len(self.node_names)} 个节点")

        # ---- 步骤 2: 获取所有话题 ----
        topic_info_list = self._node.get_topic_names_and_types()
        for topic_name, _type_list in topic_info_list:
            canonical = get_canonical_topic_name(topic_name)
            self.topic_names.add(canonical)
        print(f"[INFO]   发现 {len(self.topic_names)} 个话题")

        # ---- 步骤 3: 获取发布者/订阅者映射 ----
        for topic in self.topic_names:
            # 获取发布者
            try:
                pubs = self._node.get_publishers_info_by_topic(topic)
                self.publishers_map[topic] = [
                    get_canonical_node_name(p.node_name, p.node_namespace)
                    for p in pubs
                ]
            except Exception as e:
                print(f"[WARN] 无法获取话题 {topic} 的发布者: {e}")
                self.publishers_map[topic] = []

            # 获取订阅者
            try:
                subs = self._node.get_subscriptions_info_by_topic(topic)
                self.subscribers_map[topic] = [
                    get_canonical_node_name(s.node_name, s.node_namespace)
                    for s in subs
                ]
            except Exception as e:
                print(f"[WARN] 无法获取话题 {topic} 的订阅者: {e}")
                self.subscribers_map[topic] = []

        total_edges = sum(
            len(self.publishers_map.get(t, [])) + len(self.subscribers_map.get(t, []))
            for t in self.topic_names
        )
        print(f"[INFO]   共发现约 {total_edges} 条发布/订阅关系")


# ===========================================================================
# 过滤器
# ===========================================================================

class TopologyFilter:
    """
    根据 FilterConfig 对拓扑数据进行过滤与聚合处理。

    处理流程：
      1. 移除 hide_nodes 匹配的节点
      2. 移除 hide_topics 匹配的话题
      3. 对聚合组进行节点合并
    """

    def __init__(self, config: FilterConfig):
        self.config = config

        # 构建聚合组查找表: node_full_name -> group_dict
        self.node_to_group: Dict[str, dict] = {}
        for group in config.aggregate_groups:
            for node_name in group.get('nodes', []):
                self.node_to_group[node_name] = group

        print(f"[INFO] 已加载 {len(config.aggregate_groups)} 个聚合组")

    def filter(
        self,
        all_nodes: Set[str],
        all_topics: Set[str],
        publishers: Dict[str, List[str]],
        subscribers: Dict[str, List[str]],
    ) -> Tuple[Set[str], Set[str], Dict[str, List[str]], Dict[str, List[str]]]:
        """
        执行过滤与聚合。

        Returns:
            (filtered_nodes, filtered_topics,
             filtered_publishers, filtered_subscribers)
             其中聚合组节点以 "__aggregate__:组名" 的虚拟名称表示
        """
        # ---- 步骤 1: 隐藏节点 ----
        visible_nodes = set()
        for n in all_nodes:
            if self.node_to_group.get(n) is not None:
                # 属于聚合组的节点，后面统一处理，先保留
                visible_nodes.add(n)
            elif not matches_any_pattern(n, self.config.hide_nodes):
                visible_nodes.add(n)

        hidden_node_count = len(all_nodes) - len(visible_nodes)
        print(f"[INFO] 隐藏了 {hidden_node_count} 个节点，剩余 {len(visible_nodes)} 个")

        # ---- 步骤 2: 隐藏话题 ----
        visible_topics = set()
        for t in all_topics:
            if not matches_any_pattern(t, self.config.hide_topics):
                visible_topics.add(t)

        hidden_topic_count = len(all_topics) - len(visible_topics)
        print(f"[INFO] 隐藏了 {hidden_topic_count} 个话题，剩余 {len(visible_topics)} 个")

        # ---- 步骤 3: 节点聚合 ----
        # 构建最终的节点集合：将聚合组内节点替换为组节点
        final_nodes = set()
        group_nodes_set = set()  # 记录哪些节点已被聚合

        # 建立聚合组名到节点列表的映射
        group_name_to_nodes: Dict[str, List[str]] = {}
        for group in self.config.aggregate_groups:
            group_name = group['name']
            group_name_to_nodes[group_name] = list(group.get('nodes', []))
            for node_name in group.get('nodes', []):
                group_nodes_set.add(node_name)

        # 非聚合组节点直接加入最终节点集合
        for n in visible_nodes:
            if n not in group_nodes_set:
                final_nodes.add(n)

        # 添加聚合组虚拟节点（使用加固前缀避免与真实节点名冲突）
        for group in self.config.aggregate_groups:
            group_virtual_name = f"{AGGREGATE_PREFIX}{group['name']}"
            final_nodes.add(group_virtual_name)

        # ---- 步骤 4: 重建连线关系（已修复严重 Bug） ----
        new_publishers: Dict[str, List[str]] = {}
        new_subscribers: Dict[str, List[str]] = {}
        skipped_internal_topics = 0  # 统计跳过的组内话题数

        for topic in visible_topics:
            topic_pubs = publishers.get(topic, [])
            topic_subs = subscribers.get(topic, [])

            # ---- 4a: 转换发布者节点名 ----
            converted_pubs = []
            for pub_node in topic_pubs:
                if pub_node not in visible_nodes:
                    continue  # 被隐藏的节点，忽略
                group = self.node_to_group.get(pub_node)
                if group:
                    converted_pubs.append(f"{AGGREGATE_PREFIX}{group['name']}")
                else:
                    converted_pubs.append(pub_node)

            # ---- 4b: 转换订阅者节点名 ----
            converted_subs = []
            for sub_node in topic_subs:
                if sub_node not in visible_nodes:
                    continue
                group = self.node_to_group.get(sub_node)
                if group:
                    converted_subs.append(f"{AGGREGATE_PREFIX}{group['name']}")
                else:
                    converted_subs.append(sub_node)

            # 如果没有任何发布者或订阅者，跳过该话题
            if not converted_pubs and not converted_subs:
                continue

            # ---- 4c: 判断是否为纯组内通信话题 ----
            # 条件：所有发布者和订阅者都以 AGGREGATE_PREFIX 开头，
            #       且属于同一个聚合组名（即所有虚拟节点名完全相同）
            all_pubs_are_aggregate = (
                len(converted_pubs) > 0 and
                all(p.startswith(AGGREGATE_PREFIX) for p in converted_pubs)
            )
            all_subs_are_aggregate = (
                len(converted_subs) > 0 and
                all(s.startswith(AGGREGATE_PREFIX) for s in converted_subs)
            )

            if all_pubs_are_aggregate and all_subs_are_aggregate:
                # 判断是否所有虚拟节点名都指向同一个聚合组
                all_virtual_names = set(converted_pubs) | set(converted_subs)
                if len(all_virtual_names) == 1:
                    # 纯组内通信话题，直接跳过，不加入拓扑图
                    skipped_internal_topics += 1
                    continue

            # ---- 4d: 去重后赋值 ----
            # 使用 dict.fromkeys 保持插入顺序并去重
            unique_pubs = list(dict.fromkeys(converted_pubs))
            unique_subs = list(dict.fromkeys(converted_subs))

            if unique_pubs or unique_subs:
                new_publishers[topic] = unique_pubs
                new_subscribers[topic] = unique_subs

        print(f"[INFO] 过滤后共 {len(final_nodes)} 个节点, {len(visible_topics)} 个话题"
              f"（其中 {len(new_publishers)} 个有连接关系）")
        if skipped_internal_topics > 0:
            print(f"[INFO] 已隐藏 {skipped_internal_topics} 个聚合组内部话题")

        return final_nodes, visible_topics, new_publishers, new_subscribers


# ===========================================================================
# Graphviz DOT 生成器
# ===========================================================================

class GraphvizGenerator:
    """
    根据过滤后的拓扑数据生成 Graphviz DOT 有向图，并渲染为 SVG。
    """

    def __init__(
        self,
        output_path: str,
        rankdir: str = 'LR',
        fontname: str = 'DejaVu Sans',
    ):
        """
        Args:
            output_path: SVG 输出文件路径
            rankdir:     图布局方向，默认 'LR'（从左到右）
            fontname:    字体名称，使用系统预装的 DejaVu Sans
        """
        self.output_path = output_path
        self.rankdir = rankdir
        self.fontname = fontname

        # 创建 Graphviz 有向图对象
        self.dot = graphviz.Digraph(
            name='ROS2_Topology',
            format='svg',
            engine='dot',
        )
        # 全局图属性（黑白灰风格，适合论文印刷）
        self.dot.attr(
            rankdir=self.rankdir,
            fontname=self.fontname,
            label='ROS2 节点拓扑图',
            labelloc='t',
            fontsize='16',
            fontcolor='black',
            dpi='150',
            nodesep='0.5',
            ranksep='1.0',
        )

        # 全局节点默认样式
        self.dot.attr('node', **STYLE_NODE)
        # 全局边默认样式
        self.dot.attr('edge', **STYLE_EDGE)

    def add_node(self, name: str, is_aggregate: bool = False):
        """
        向图中添加一个节点。

        Args:
            name:         节点显示名称
            is_aggregate: 是否为聚合组节点
        """
        if is_aggregate:
            # 聚合组节点使用特殊样式
            self.dot.node(name, **STYLE_AGGREGATE)
        else:
            self.dot.node(name)

    def add_topic(self, name: str):
        """
        向图中添加一个话题节点（椭圆样式）。

        Args:
            name: 话题名称
        """
        self.dot.node(name, **STYLE_TOPIC)

    def add_edge(self, from_node: str, to_node: str, topic_name: str = ''):
        """
        向图中添加一条有向边。

        Args:
            from_node:  发布者节点名
            to_node:    话题节点名（或订阅者节点名）
            topic_name: 话题标签（可选）
        """
        self.dot.edge(from_node, to_node, label=topic_name)

    def generate(
        self,
        nodes: Set[str],
        topics: Set[str],
        publishers: Dict[str, List[str]],
        subscribers: Dict[str, List[str]],
    ):
        """
        根据过滤后的拓扑数据生成 DOT 图并渲染。

        Args:
            nodes:       最终节点集合（含聚合组虚拟节点）
            topics:      最终话题集合
            publishers:  话题 -> 发布者列表
            subscribers: 话题 -> 订阅者列表
        """
        print("[INFO] 正在生成 Graphviz DOT 图...")

        # ---- 添加节点 ----
        for node_name in sorted(nodes):
            is_aggregate = node_name.startswith(AGGREGATE_PREFIX)
            if is_aggregate:
                # 提取聚合组显示名称（去掉前缀）
                display_name = node_name[len(AGGREGATE_PREFIX):]
            else:
                display_name = node_name
            self.add_node(display_name, is_aggregate=is_aggregate)

        # ---- 添加话题节点 ----
        for topic_name in sorted(topics):
            self.add_topic(topic_name)

        # ---- 添加连线 ----
        # 标准"节点 -> 话题 -> 节点"模式：
        #   发布者节点 ---> 话题节点 ---> 订阅者节点
        for topic in sorted(topics):
            pubs = publishers.get(topic, [])
            subs = subscribers.get(topic, [])

            # 去重发布者和订阅者
            unique_pubs = list(dict.fromkeys(pubs))
            unique_subs = list(dict.fromkeys(subs))

            for pub in unique_pubs:
                pub_display = (
                    pub[len(AGGREGATE_PREFIX):]
                    if pub.startswith(AGGREGATE_PREFIX) else pub
                )
                self.add_edge(pub_display, topic)

            for sub in unique_subs:
                sub_display = (
                    sub[len(AGGREGATE_PREFIX):]
                    if sub.startswith(AGGREGATE_PREFIX) else sub
                )
                self.add_edge(topic, sub_display)

        # ---- 渲染为 SVG ----
        output_dir = os.path.dirname(self.output_path) or '.'
        output_basename = os.path.splitext(os.path.basename(self.output_path))[0]

        print(f"[INFO] 正在渲染 SVG 到: {self.output_path}")
        try:
            # graphviz 会自动添加 .svg 扩展名，所以这里传不带扩展名的路径
            self.dot.render(
                filename=output_basename,
                directory=output_dir,
                cleanup=True,  # 删除中间 DOT 文件
                format='svg',
            )
            print(f"[INFO] ✅ SVG 拓扑图已生成: {self.output_path}")
        except Exception as e:
            print(f"[ERROR] 渲染 SVG 失败: {e}")
            # 尝试回退：保存 DOT 源文件
            dot_path = os.path.join(output_dir, output_basename + '.dot')
            with open(dot_path, 'w', encoding='utf-8') as f:
                f.write(self.dot.source)
            print(f"[INFO] DOT 源文件已保存到: {dot_path}")
            print("[WARN] 请手动使用以下命令生成 SVG:")
            print(f"       dot -Tsvg {dot_path} -o {self.output_path}")


# ===========================================================================
# 主流程
# ===========================================================================

class GraphFilterNode(Node):
    """
    ros2_graph_filter 主节点。

    该类继承自 rclpy.node.Node，负责：
      1. 初始化 ROS2 节点
      2. 采集拓扑信息
      3. 加载配置并执行过滤
      4. 生成 SVG 输出
      5. 自动退出
    """

    def __init__(self, config_path: str, output_path: str):
        """
        Args:
            config_path:  YAML 配置文件路径
            output_path:  SVG 输出文件路径
        """
        super().__init__('ros2_graph_filter')
        self.config_path = config_path
        self.output_path = output_path

    def run(self):
        """执行完整的工作流程。"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('ros2_graph_filter —— ROS2 节点拓扑图生成器')
        self.get_logger().info('=' * 60)

        # ---- 步骤 1: 加载 YAML 配置 ----
        self.get_logger().info(f'加载配置文件: {self.config_path}')
        config = FilterConfig.from_yaml(self.config_path)
        self.get_logger().info(
            f'  隐藏节点规则: {len(config.hide_nodes)} 条'
        )
        self.get_logger().info(
            f'  隐藏话题规则: {len(config.hide_topics)} 条'
        )
        self.get_logger().info(
            f'  聚合组:        {len(config.aggregate_groups)} 个'
        )

        # ---- 步骤 2: 采集拓扑信息 ----
        collector = GraphTopologyCollector(self)
        collector.collect()

        # ---- 步骤 3: 执行过滤与聚合 ----
        filter_engine = TopologyFilter(config)
        final_nodes, final_topics, final_pubs, final_subs = filter_engine.filter(
            collector.node_names,
            collector.topic_names,
            collector.publishers_map,
            collector.subscribers_map,
        )

        # ---- 步骤 4: 生成 Graphviz DOT 并渲染 SVG ----
        generator = GraphvizGenerator(self.output_path)
        generator.generate(final_nodes, final_topics, final_pubs, final_subs)

        self.get_logger().info('=' * 60)
        self.get_logger().info('✅ 拓扑图生成完成！')
        self.get_logger().info(f'   输出文件: {self.output_path}')
        self.get_logger().info('   可直接用浏览器打开查看')
        self.get_logger().info('=' * 60)


# ===========================================================================
# 入口函数
# ===========================================================================

def main():
    """
    程序主入口。

    解析命令行参数，初始化 rclpy，创建节点并运行，
    完成后自动关闭。
    """
    parser = argparse.ArgumentParser(
        description='ROS2 节点拓扑图生成器 —— 根据过滤规则生成 SVG 拓扑图',
    )
    parser.add_argument(
        'config',
        nargs='?',
        default=None,
        help='YAML 配置文件路径（默认使用包内置的 config/filter.yaml）',
    )
    parser.add_argument(
        'output',
        nargs='?',
        default='ros2_topology.svg',
        help='输出 SVG 文件路径（默认: ros2_topology.svg）',
    )
    args = parser.parse_args()

    # 确定配置文件路径
    if args.config:
        config_path = args.config
    else:
        # 默认使用包内置配置文件
        # 通过 ament_index 查找包的 share 目录
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('ros2_graph_filter')
            config_path = os.path.join(pkg_share, 'config', 'filter.yaml')
        except ImportError:
            # 如果找不到 ament_index，回退到相对路径
            config_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'filter.yaml'
            )
            config_path = os.path.abspath(config_path)

    output_path = args.output

    print(f"[INFO] 配置文件: {config_path}")
    print(f"[INFO] 输出文件: {output_path}")

    # 初始化 rclpy
    rclpy.init(args=sys.argv)

    try:
        # 创建节点并运行
        node = GraphFilterNode(config_path, output_path)
        node.run()
    except Exception as e:
        print(f"[ERROR] 运行失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 销毁节点并关闭 rclpy
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
