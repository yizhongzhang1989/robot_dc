#!/usr/bin/env python3
"""
OBJ to Wireframe Viewer
=======================

This script reads an OBJ file, extracts all edges, and displays them
as a wireframe in a 3D window using matplotlib.
"""

import numpy as np


class WireframeBuilder:
    """
    A class for building wireframe representations from OBJ files.
    
    This class provides methods to:
    - Read OBJ files and extract vertices and faces
    - Extract feature edges from faces
    - Group edges into connected wires
    - Subdivide long edges
    - Visualize wires in 3D
    
    Attributes:
        vertices: Nx3 numpy array of vertex coordinates
        faces: List of face definitions (list of vertex indices)
    """
    
    def __init__(self):
        """Initialize an empty WireframeBuilder."""
        self.vertices = None
        self.faces = None
    
    def generate_wires(self, obj_path, feature_edge_only_flag=True, group_flag=True, max_edge_length=None):
        """
        Main entry point for generating wires from an OBJ file.
        
        Args:
            obj_path: Path to the OBJ file
            feature_edge_only_flag: If True, extract only feature edges; otherwise extract all edges (default: True)
            group_flag: If True, group edges into connected wires; otherwise return individual edges (default: True)
            max_edge_length: Maximum allowed edge length; if specified, subdivide longer edges (default: None)
            
        Returns:
            List of wires, where each wire is a list of edges with coordinates
        """
        # Load OBJ file
        self._read_obj_file(obj_path)
        
        if feature_edge_only_flag:
            # Extract feature edges (excluding coplanar shared edges)
            edges_to_use = self._extract_feature_edges_from_faces()
        else:
            # Extract all edges
            edges_to_use = self._extract_edges_from_faces()
        
        # Convert edges to wires with coordinates
        wires = self._generate_wires_from_edges(edges_to_use, group_flag=group_flag)
        
        # Limit edge length if specified
        if max_edge_length is not None:
            wires = self._limit_wire_edge_length(wires, max_edge_length)
        
        return wires
    
    def _read_obj_file(self, filepath):
        """
        Read an OBJ file and extract vertices and faces.
        
        Args:
            filepath: Path to the OBJ file
            
        Returns:
            self (for method chaining)
        """
        vertices = []
        faces = []
        
        with open(filepath, 'r') as f:
            for line in f:
                line = line.strip()
                
                if not line or line.startswith('#'):
                    continue
                
                parts = line.split()
                if not parts:
                    continue
                
                if parts[0] == 'v':
                    x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                    vertices.append([x, y, z])
                
                elif parts[0] == 'f':
                    face_vertices = []
                    for vertex_str in parts[1:]:
                        vertex_index = int(vertex_str.split('/')[0])
                        face_vertices.append(vertex_index - 1)
                    faces.append(face_vertices)
        
        self.vertices = np.array(vertices)
        self.faces = faces
        return self
    
    def _extract_edges_from_faces(self):
        """
        Extract unique edges from face definitions.
        
        Returns:
            List of edges, where each edge is a tuple (v1_idx, v2_idx) with v1_idx < v2_idx
        """
        edges = set()
        
        for face in self.faces:
            num_vertices = len(face)
            for i in range(num_vertices):
                v1 = face[i]
                v2 = face[(i + 1) % num_vertices]
                edge = (min(v1, v2), max(v1, v2))
                edges.add(edge)
        
        return list(edges)
    
    def _compute_face_normal(self, face):
        """
        Compute the normal vector of a face using Newell's method.
        
        Args:
            face: List of vertex indices defining the face
            
        Returns:
            Normalized normal vector as numpy array [nx, ny, nz]
        """
        normal = np.zeros(3)
        n = len(face)
        
        for i in range(n):
            v1 = self.vertices[face[i]]
            v2 = self.vertices[face[(i + 1) % n]]
            
            normal[0] += (v1[1] - v2[1]) * (v1[2] + v2[2])
            normal[1] += (v1[2] - v2[2]) * (v1[0] + v2[0])
            normal[2] += (v1[0] - v2[0]) * (v1[1] + v2[1])
        
        length = np.linalg.norm(normal)
        if length > 1e-10:
            normal = normal / length
        
        return normal
    
    def _extract_feature_edges_from_faces(self, angle_threshold=1e-6):
        """
        Extract feature edges from face definitions.
        
        Args:
            angle_threshold: Threshold for considering faces parallel (default: 1e-6)
            
        Returns:
            List of feature edges, where each edge is a tuple (v1_idx, v2_idx)
        """
        edge_to_faces = {}
        
        for face_idx, face in enumerate(self.faces):
            num_vertices = len(face)
            for i in range(num_vertices):
                v1 = face[i]
                v2 = face[(i + 1) % num_vertices]
                edge = (min(v1, v2), max(v1, v2))
                
                if edge not in edge_to_faces:
                    edge_to_faces[edge] = []
                edge_to_faces[edge].append(face_idx)
        
        face_normals = [self._compute_face_normal(face) for face in self.faces]
        
        feature_edges = []
        
        for edge, face_indices in edge_to_faces.items():
            if len(face_indices) == 1:
                feature_edges.append(edge)
            elif len(face_indices) == 2:
                normal1 = face_normals[face_indices[0]]
                normal2 = face_normals[face_indices[1]]
                dot_product = np.dot(normal1, normal2)
                
                if abs(abs(dot_product) - 1.0) > angle_threshold:
                    feature_edges.append(edge)
            else:
                feature_edges.append(edge)
        
        return feature_edges
    
    def _group_edges(self, edges):
        """
        Group edges into connected components using Union-Find algorithm.
        
        Args:
            edges: List of (v1_idx, v2_idx) tuples defining edges
            
        Returns:
            List of edge groups, where each group is a list of connected edges
        """
        if not edges:
            return []
        
        parent = {}
        
        def find(v):
            if v not in parent:
                parent[v] = v
            if parent[v] != v:
                parent[v] = find(parent[v])
            return parent[v]
        
        def union(v1, v2):
            root1 = find(v1)
            root2 = find(v2)
            if root1 != root2:
                parent[root2] = root1
        
        for v1_idx, v2_idx in edges:
            union(v1_idx, v2_idx)
        
        from collections import defaultdict
        component_edges = defaultdict(list)
        for edge in edges:
            v1_idx, v2_idx = edge
            root = find(v1_idx)
            component_edges[root].append(edge)
        
        return list(component_edges.values())
    
    def _generate_wires_from_edges(self, edges, group_flag=True):
        """
        Convert edges to wires with coordinates and sort by total length.
        
        Args:
            edges: Flat list of (v1_idx, v2_idx) tuples defining edges
            group_flag: If True, group edges first and return grouped wires. 
                       If False, return flat list of individual edge wires.
            
        Returns:
            If group_flag=True: List of wires sorted by total length (descending), 
                               where each wire is a list of edges with coordinates.
            If group_flag=False: Flat list of edges with coordinates (not grouped into wires).
        """
        if not edges:
            return []
        
        if group_flag:
            # Group edges into connected components
            edge_groups = self._group_edges(edges)
            
            wires_with_lengths = []
            
            for edge_group in edge_groups:
                total_length = 0.0
                wire = []
                
                for v1_idx, v2_idx in edge_group:
                    v1 = self.vertices[v1_idx]
                    v2 = self.vertices[v2_idx]
                    
                    edge_length = np.linalg.norm(v2 - v1)
                    total_length += edge_length
                    
                    wire.append([v1.tolist(), v2.tolist()])
                
                wires_with_lengths.append((total_length, wire))
            
            wires_with_lengths.sort(key=lambda x: x[0], reverse=True)
            
            return [wire for length, wire in wires_with_lengths]
        else:
            # Return flat list of individual edges with coordinates
            wires = []
            for v1_idx, v2_idx in edges:
                v1 = self.vertices[v1_idx]
                v2 = self.vertices[v2_idx]
                wires.append([v1.tolist(), v2.tolist()])
            
            return wires
    
    def _limit_wire_edge_length(self, wires, max_edge_len):
        """
        Break long edges into shorter segments.
        
        Args:
            wires: Either a flat list of edges [[v1,v2], [v3,v4], ...] or 
                   a list of wires [[[v1,v2], [v3,v4]], [[v5,v6]], ...]
            max_edge_len: Maximum allowed edge length
            
        Returns:
            Same format as input (flat or grouped)
        """
        if not wires:
            return []
        
        # Detect if wires is grouped or flat
        # Grouped: wires[0][0] is an edge [[x1,y1,z1], [x2,y2,z2]]
        # Flat: wires[0] is an edge [[x1,y1,z1], [x2,y2,z2]]
        is_grouped = len(wires[0]) > 0 and isinstance(wires[0][0], list) and len(wires[0][0]) == 2
        
        if is_grouped:
            # Process grouped wires
            limited_wires = []
            
            for wire in wires:
                limited_wire = []
                
                for edge in wire:
                    v1 = np.array(edge[0])
                    v2 = np.array(edge[1])
                    
                    edge_vec = v2 - v1
                    edge_length = np.linalg.norm(edge_vec)
                    
                    if edge_length <= max_edge_len:
                        limited_wire.append(edge)
                    else:
                        num_segments = int(np.ceil(edge_length / max_edge_len))
                        
                        for i in range(num_segments):
                            t_start = i / num_segments
                            t_end = (i + 1) / num_segments
                            
                            segment_v1 = v1 + t_start * edge_vec
                            segment_v2 = v1 + t_end * edge_vec
                            
                            limited_wire.append([segment_v1.tolist(), segment_v2.tolist()])
                
                limited_wires.append(limited_wire)
            
            return limited_wires
        else:
            # Process flat list of edges
            limited_edges = []
            
            for edge in wires:
                v1 = np.array(edge[0])
                v2 = np.array(edge[1])
                
                edge_vec = v2 - v1
                edge_length = np.linalg.norm(edge_vec)
                
                if edge_length <= max_edge_len:
                    limited_edges.append(edge)
                else:
                    num_segments = int(np.ceil(edge_length / max_edge_len))
                    
                    for i in range(num_segments):
                        t_start = i / num_segments
                        t_end = (i + 1) / num_segments
                        
                        segment_v1 = v1 + t_start * edge_vec
                        segment_v2 = v1 + t_end * edge_vec
                        
                        limited_edges.append([segment_v1.tolist(), segment_v2.tolist()])
            
            return limited_edges
    
    def get_vertices(self):
        """
        Get the loaded vertices.
        
        Returns:
            Nx3 numpy array of vertex coordinates.
        """
        return self.vertices
    
    def get_faces(self):
        """
        Get the loaded faces.
        
        Returns:
            List of face definitions (list of vertex indices).
        """
        return self.faces


def draw_wires_3d(wires, title="Wires View"):
    """
    Display wires in a 3D matplotlib window.
    
    Args:
        wires: Either a flat list of edges [[v1,v2], [v3,v4], ...] or 
               a list of grouped wires [[[v1,v2], [v3,v4]], [[v5,v6]], ...]
        title: Title for the plot window
    """
    if not wires:
        print("No wires to display")
        return
    
    import matplotlib.pyplot as plt
    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = ['r', 'g', 'b', 'c', 'm', 'y', 'orange', 'purple', 'brown', 'pink']
    
    all_points = []
    
    # Detect if wires is grouped or flat
    # Grouped: wires[0][0] is an edge [[x1,y1,z1], [x2,y2,z2]]
    # Flat: wires[0] is an edge [[x1,y1,z1], [x2,y2,z2]]
    is_grouped = len(wires[0]) > 0 and isinstance(wires[0][0], list) and len(wires[0][0]) == 2
    
    if is_grouped:
        # Grouped wires: use one color per wire
        for wire_idx, wire in enumerate(wires):
            color = colors[wire_idx % len(colors)]
            
            for edge in wire:
                v1 = np.array(edge[0])
                v2 = np.array(edge[1])
                
                ax.plot([v1[0], v2[0]], 
                        [v1[1], v2[1]], 
                        [v1[2], v2[2]], 
                        color=color, linewidth=1.0)
                
                all_points.append(v1)
                all_points.append(v2)
    else:
        # Flat list of edges: change color for each edge
        for edge_idx, edge in enumerate(wires):
            color = colors[edge_idx % len(colors)]
            
            v1 = np.array(edge[0])
            v2 = np.array(edge[1])
            
            ax.plot([v1[0], v2[0]], 
                    [v1[1], v2[1]], 
                    [v1[2], v2[2]], 
                    color=color, linewidth=1.0)
            
            all_points.append(v1)
            all_points.append(v2)
    
    all_points = np.array(all_points)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(title)
    
    max_range = np.array([
        all_points[:, 0].max() - all_points[:, 0].min(),
        all_points[:, 1].max() - all_points[:, 1].min(),
        all_points[:, 2].max() - all_points[:, 2].min()
    ]).max() / 2.0
    
    mid_x = (all_points[:, 0].max() + all_points[:, 0].min()) * 0.5
    mid_y = (all_points[:, 1].max() + all_points[:, 1].min()) * 0.5
    mid_z = (all_points[:, 2].max() + all_points[:, 2].min()) * 0.5
    
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    ax.text2D(0.05, 0.95, f"Total wires: {len(wires)}", 
              transform=ax.transAxes, fontsize=10, verticalalignment='top')
    
    plt.show()


def main():
    """Main function demonstrating WireframeBuilder usage."""
    # Parse command line arguments
    import argparse
    import os

    parser = argparse.ArgumentParser(description='Extract and visualize edges from OBJ file')
    parser.add_argument('obj_file', nargs='?', default='GB200_simplified.obj',
                        help='Path to OBJ file (default: GB200_simplified.obj)')
    parser.add_argument('--title', default='Feature Edge Wires',
                        help='Window title for visualization (default: Feature Edge Wires)')
    args = parser.parse_args()
    
    # Check if file exists
    if not os.path.exists(args.obj_file):
        print(f"Error: OBJ file '{args.obj_file}' not found!")
        print(f"Please place the file in the current directory: {os.getcwd()}")
        return 1
    
    print(f"Reading OBJ file: {args.obj_file}")
    
    # Build wireframe using WireframeBuilder class
    try:
        builder = WireframeBuilder()
        
        # Generate wires with feature edges, grouping, and edge length limit
        max_edge_length = 0.05
        print(f"Generating wires from {args.obj_file}...")
        wires = builder.generate_wires(
            obj_path=args.obj_file,
            feature_edge_only_flag=True,
            group_flag=True,
            max_edge_length=max_edge_length
        )
        
        print(f"Loaded {len(builder.vertices)} vertices and {len(builder.faces)} faces")
        
        # Detect if wires are grouped or flat
        is_grouped = len(wires[0]) > 0 and isinstance(wires[0][0], list) and len(wires[0][0]) == 2
        
        if is_grouped:
            print(f"Generated {len(wires)} grouped wire sets")
            
            # Print wire information for grouped wires
            for i, wire in enumerate(wires):
                total_length = sum(np.linalg.norm(np.array(edge[1]) - np.array(edge[0])) for edge in wire)
                max_edge_in_wire = max(np.linalg.norm(np.array(edge[1]) - np.array(edge[0])) for edge in wire)
                print(f"  Wire {i+1}: {len(wire)} edges, total length: {total_length:.4f}, max edge: {max_edge_in_wire:.4f}")
        else:
            print(f"Generated {len(wires)} individual edges (ungrouped)")
            
            # Print edge information for flat list
            total_length = sum(np.linalg.norm(np.array(edge[1]) - np.array(edge[0])) for edge in wires)
            max_edge_length = max(np.linalg.norm(np.array(edge[1]) - np.array(edge[0])) for edge in wires)
            print(f"  Total edges: {len(wires)}, total length: {total_length:.4f}, max edge: {max_edge_length:.4f}")
        
        # Display wires in 3D window
        print("\nDisplaying wires in 3D window...")
        draw_wires_3d(wires, title=args.title)
        
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
