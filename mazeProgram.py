import maze
import generate_maze
import sys
import random
import json


class Node():
    def __init__(self, index = 0, parent = None):
        self.index = index
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0


#######################################################  SOLVE WITH A*  #####################################################################
def Astar(m):

    if int(maze.x1) == 0 :
        if int(maze.y1) == 0 :
            start = 0
        else:
            start = ((int(maze.y1) - 1) * int(maze.n))
    elif int(maze.y1) == 0 :
        start = (int(maze.x1) - 1)
    else:
        start = (int(maze.x1)-1) + ((int(maze.y1) -1 )* int(maze.n))

    end = (int(maze.x2)-1) + ((int(maze.y2) -1 )* int(maze.n))

    start_node = Node(start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(end)
    end_node.g = end_node.h = end_node.f = 0

    fringe = []
    close_list = []

    fringe.append(start_node)


    # for start node
    unvisited_neighbor = m.cell_neighbors(start)
    for item in unvisited_neighbor:
        nei, compass_index = item
        #x1, y1 = m.x_y(nei)
        nei_node = Node(nei, start_node)
        nei_node.g = start_node.g + 1
        nei_node.h = ((int(maze.x1) - int(maze.x2)) ** 2) + ((int(maze.y1) - int(maze.y2)) ** 2)
        nei_node.f = nei_node.h + nei_node.g

        if nei not in fringe:
            fringe.append(nei_node)
            fringe.sort(key=lambda x: x.f,reverse=False)
        else:
            for index, item in enumerate(fringe):
                if nei_node.f < item.f:
                    fringe.remove(item)
                    fringe.append(nei_node)
                    fringe.sort(key=lambda x: x.f,reverse=False)
#**************************
    while len(fringe) > 0:
        current_node = fringe.pop(0)
        for obj in unvisited_neighbor:
            cell, in_direction = obj
            if current_node.index == cell:
                m.bfs_visit_cell(cell, in_direction)
        m.refresh_maze_view()

        close_list.append(current_node)
        # visit and draw it
        if current_node.index == end_node.index:
            m.reconstruct(current_node)
            m.state = "idle"
            #m.backtrack()

        # for current node
        unvisited_neighbor = m.cell_neighbors(current_node.index)

        for an_item in unvisited_neighbor:
            # if nei not in close_list:
            nei, compass_index = an_item
            x1, y1 = m.x_y(nei)
            nei_node = Node(nei, current_node)
            nei_node.g = current_node.g + 1
            nei_node.h = ((x1 - int(maze.x2)) ** 2) + ((y1 - int(maze.y2)) ** 2)
            nei_node.f = nei_node.h + nei_node.g

            if nei not in fringe:
                fringe.append(nei_node)
                fringe.sort(key=lambda x: x.f,reverse=False)
            else:
                for index, item in enumerate(fringe):
                    if nei_node.f < item.f:
                        fringe.remove(item)
                        fringe.append(nei_node)
                        fringe.sort(key=lambda x: x.f,reverse=False)
                    else:
                        unvisited_neighbor.remove((nei, compass_index))


#############################################  BFS & DFS   #############################################################
BIT_SOLUTION = 0b0000010010010110
# Solve maze using Pre-Order DFS algorithm, terminate with solution
def solve_dfs(m):
    stack = []
    if int(maze.x1) == 0 :
        if int(maze.y1) == 0 :
            current_cell = 0
        else:
            current_cell = ((int(maze.y1) - 1) * int(maze.n))
    elif int(maze.y1) == 0 :
        current_cell = (int(maze.x1) - 1)
    else:
        current_cell = (int(maze.x1)-1) + ((int(maze.y1) -1 )* int(maze.n))

    visited_cells = 1

    while current_cell != (int(maze.x2)-1) + ((int(maze.y2) -1 )* int(maze.n)):
        #print(current_cell)
        unvisited_neighbors = m.cell_neighbors(current_cell)
        if len(unvisited_neighbors) >= 1:
            # choose random neighbor to be new cell
            new_cell_index = random.randint(0, len(unvisited_neighbors) - 1)
            new_cell, compass_index = unvisited_neighbors[new_cell_index]
            # knock down wall between it and current cell using visited_cell
            m.visit_cell(current_cell, new_cell, compass_index)
            # push current cell to stack
            stack.append(current_cell)
            # set current cell to new cell
            current_cell = new_cell
            # add 1 to visited cells
            visited_cells += 1
        else:
            m.backtrack(current_cell)
            current_cell = stack.pop()
        m.refresh_maze_view()
    m.state = 'idle'


# Solve maze using BFS algorithm, terminate with solution
def solve_bfs(m):

    queue = []
    if int(maze.x1) == 0:
        if int(maze.y1) == 0:
            cur_cell = 0
        else:
            cur_cell = ((int(maze.y1) - 1) * int(maze.n))
    elif int(maze.y1) == 0 :
        cur_cell = (int(maze.x1) - 1)
    else:
        cur_cell = ( int(maze.x1) - 1 ) + (( int(maze.y1) - 1 ) * int(maze.n))

    #(int(maze.y1)-1) * m.w_cells + (int(maze.x1)-1)
    in_direction = 0b0000
    visited_cells = 0
    queue.insert(0, (cur_cell, in_direction))
    # while current cell is not goal and queue is not empty
    while not (cur_cell == ((int(maze.x2)-1) + ((int(maze.y2) -1 )* int(maze.n)))) and len(queue) > 0:
        cur_cell, in_direction = queue.pop()
        m.bfs_visit_cell(cur_cell, in_direction)
        visited_cells += 1
        m.refresh_maze_view()
        neighbors = m.cell_neighbors(cur_cell)
        for neighbor in neighbors:
            queue.insert(0, neighbor)
    m.reconstruct_solution(cur_cell)
    m.state = "idle"
    #idle : nothing to work for, nothing to do


def main(solver='dfs'):
    current_maze = maze.Maze('create')
    generate_maze.create_dfs(current_maze)
    if solver == 'dfs':
        solve_dfs(current_maze)
    elif solver == 'bfs':
        solve_bfs(current_maze)
    elif solver == 'astar':
        Astar(current_maze)

    # Asking for Edit File.
    print('Do you want to edit the parameters? (yes/no)')
    ans = input()
    if ans == 'yes':
        f = open('options.txt', 'w+')
        print('Enter A Number For Dimensions: ')
        n = input()
        print('Enter the starting position(2 number with a space ): ')
        x1, y1 = input().split()
        print('Enter the finishing position(2 number with a space ): ')
        x2, y2 = input().split()
        data = {'Dimensions': n,
                'x1': x1,
                'x2': x2,
                'y1': y1,
                'y2': y2
                }
        json.dump(data, f)
        f.close()
        print('Your File Has Been Edited. Exit And Run again. ')

    elif ans == 'no':
        print('OK! Thanks.')
    else:
        print('INVALID INPUT!')

    while 1:
        maze.check_for_exit()
    return

if __name__ == '__main__':
    print("Choose The Solver's Algorithm. 'bfs' or 'dfs' or 'astar'? (type here) ")
    solver = input()
    if len(sys.argv) > 1:
        main(sys.argv[1])
    else:
        main(solver)