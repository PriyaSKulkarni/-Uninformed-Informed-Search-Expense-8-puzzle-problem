# Uninformed-Informed-Search-Expense-8-puzzle-problem
CSE 5360 : Artificial Intelligence


Designed an intelligent agent to tackle the Expense 8 puzzle, a modified 8 puzzle variant where each tile's number denotes the moving cost within a 3x3 grid. Successfully crafted an optimal sequence for tile movement, showcasing strong algorithmic and artificial intelligence skills.Designed an intelligent agent to tackle the Expense 8 puzzle, a modified 8 puzzle variant where each tile's number denotes the moving cost within a 3x3 grid. Successfully crafted an optimal sequence for tile movement, showcasing strong algorithmic and artificial intelligence skills.

Skills: Artificial Intelligence (AI) · Problem Solving · Algorithm · Python (Programming Language)

Programming language used for this task: Python 3.11.5

How the code is structured:
1. All the methods are implemented as functions 
2. later depending on the command prompt input the method is called respectively

How to run the code:
1. open visual studio.
2. open the folder which has the expense_8_puzzle.py input.txt output.txt
3. open the command prompt
4. run the command 
		"python expense_8_puzzle.py input.txt output.txt astar true" for A* method
		"python expense_8_puzzle.py input.txt output.txt greedy true" for Greedy search method
		"python expense_8_puzzle.py input.txt output.txt ids true" for Iterative Deepening Search
		"python expense_8_puzzle.py input.txt output.txt dls true" for Depth Limited Search 
			this asks for the depth to be inputed from the user
		"python expense_8_puzzle.py input.txt output.txt ufs true" for Uniform Cost Search
		"python expense_8_puzzle.py input.txt output.txt dfs true" for Depth first search
		"python expense_8_puzzle.py input.txt output.txt bfs true" for Breadth first search

if dump_flag is set to "true" --> dump_flag will be created in the current folder and dump_flag output can be seen inside the dump_flag

if dump_flag is set to "false" ==> dump_flag will not be created in the current folder 

Heuristic used: Manhanttan distance

NOTE PLEASE ADD THE INPUT and OUTPUT FILES(.txt) in the same directory.	
