import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations

# Define perimeter positions (1-12) on a 4x4 grid (0-indexed)
PERIMETER_POSITIONS = [
    (0,0), (0,1), (0,2), (0,3),
    (1,3), (2,3),
    (3,3), (3,2), (3,1), (3,0),
    (2,0), (1,0)
]

# Predefined 20 unique patterns (bead indices 0-11)
PATTERNS = [
    [0, 5, 8, 10],                 # Pattern 1 (5 beads)
    [2, 5, 7, 10, 11],              # Pattern 2 (5 beads)
    [0, 3, 5, 7, 10, 11],          # Pattern 3 (6 beads)
    [2, 3, 5, 9, 10],               # Pattern 4 (5 beads)
    [0, 4, 6, 8, 11],              # Pattern 5 (5 beads)
    [3, 4, 8, 10, 11],              # Pattern 6 (5 beads
    [0, 1, 6, 7, 9, 11],           # Pattern 7 (6 beads)
    [1, 5, 6, 8, 10],              # Pattern 8 (5 beads)
    [3, 4, 5, 9, 11],              # Pattern 9 (5 beads)
    [0, 2, 4, 8, 10, 11],          # Pattern 10 (6 beads)
    [1, 3, 5, 7, 9],               # Pattern 11 (5 beads)
    [2, 4, 6, 8, 10],              # Pattern 12 (5 beads)
    [0, 1, 4, 7, 9, 11],           # Pattern 13 (6 beads)
    [3, 5, 6, 8, 10],              # Pattern 14 (5 beads)
    [1, 3, 7, 8, 11],              # Pattern 15 (5 beads)
    [1, 3, 4, 8, 9, 10],           # Pattern 16 (6 beads)
    [2, 4, 7, 9, 11],              # Pattern 17 (5 beads)
    [0, 3, 5, 8, 10],              # Pattern 18 (5 beads)
    [1, 2, 6, 9, 11],              # Pattern 19 (5 beads)
    [4, 5, 7, 8, 10, 11]           # Pattern 20 (6 beads)
]

def generate_grid(pattern_idx):
    """Create a 4x4 grid with beads at specified perimeter positions"""
    grid = np.zeros((4,4), dtype=int)
    for pos_idx in PATTERNS[pattern_idx]:
        row, col = PERIMETER_POSITIONS[pos_idx]
        grid[row,col] = 1
    return grid

def print_pattern_ascii(grid):
    """ASCII visualization of a single pattern"""
    for row in grid:
        print(' '.join('●' if x else '○' for x in row))
    print()

def plot_all_patterns():
    """Matplotlib visualization of all patterns"""
    plt.figure(figsize=(15, 12))
    for i in range(20):
        plt.subplot(4, 5, i+1)
        grid = generate_grid(i)
        plt.imshow(grid, cmap='binary', vmin=0, vmax=1)
        plt.title(f'Pattern {i+1}')
        plt.axis('off')
    plt.tight_layout()
    plt.show()

def verify_uniqueness():
    """Verify no patterns are rotations/reflections of each other"""
    all_patterns = []
    for i in range(20):
        grid = generate_grid(i)
        all_patterns.append(grid)
    
    # Check all pairs
    for i in range(20):
        for j in range(i+1, 20):
            if is_similar(all_patterns[i], all_patterns[j]):
                print(f"WARNING: Pattern {i+1} and {j+1} are not unique!")
    print("Uniqueness verification complete.")

def is_similar(grid1, grid2):
    """Check if two grids are rotations/reflections of each other"""
    for rotation in [0, 1, 2, 3]:  # 0°, 90°, 180°, 270°
        rotated = np.rot90(grid1, rotation)
        if np.array_equal(rotated, grid2):
            return True
        if np.array_equal(np.fliplr(rotated), grid2):
            return True
    return False

# Main execution
if __name__ == "__main__":
    print("Generating all 20 unique patterns...\n")
    
    # ASCII print first 3 patterns as example
    for i in range(20):
        print(f"Pattern {i+1}:")
        print_pattern_ascii(generate_grid(i))
    
    # Visualize all patterns with matplotlib
    plot_all_patterns()
    
    # Verify pattern uniqueness
    verify_uniqueness()