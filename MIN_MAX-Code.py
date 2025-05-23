import math


PLAYER_X = 'X'
PLAYER_O = 'O'
EMPTY = ' '


def check_winner(board):
    # Check rows, columns, and diagonals
    for i in range(3):
        if board[i][0] == board[i][1] == board[i][2] != EMPTY:
            return board[i][0]
        if board[0][i] == board[1][i] == board[2][i] != EMPTY:
            return board[0][i]
    if board[0][0] == board[1][1] == board[2][2] != EMPTY:
        return board[0][0]
    if board[0][2] == board[1][1] == board[2][0] != EMPTY:
        return board[0][2]
    return None


def is_full(board):
    return all(cell != EMPTY for row in board for cell in row)


def minimax(board, depth, is_maximizing):
    winner = check_winner(board)
    if winner == PLAYER_X:
        return -1
    elif winner == PLAYER_O:
        return 1
    elif is_full(board):
        return 0

    if is_maximizing:
        best_score = -math.inf
        for i in range(3):
            for j in range(3):
                if board[i][j] == EMPTY:
                    board[i][j] = PLAYER_O
                    score = minimax(board, depth + 1, False)
                    board[i][j] = EMPTY
                    best_score = max(score, best_score)
        return best_score
    else:
        best_score = math.inf
        for i in range(3):
            for j in range(3):
                if board[i][j] == EMPTY:
                    board[i][j] = PLAYER_X
                    score = minimax(board, depth + 1, True)
                    board[i][j] = EMPTY
                    best_score = min(score, best_score)
        return best_score


def find_best_move(board):
    best_score = -math.inf
    best_move = (-1, -1)
    for i in range(3):
        for j in range(3):
            if board[i][j] == EMPTY:
                board[i][j] = PLAYER_O
                score = minimax(board, 0, False)
                board[i][j] = EMPTY
                if score > best_score:
                    best_score = score
                    best_move = (i, j)
    return best_move


def print_board(board):
    for row in board:
        print('|'.join(row))
        print('-' * 5)


def play_game():
    board = [[EMPTY] * 3 for _ in range(3)]
    while True:
        print_board(board)
        if is_full(board):
            print("It's a draw!")
            break

        
        x_row, x_col = map(int, input("Enter row and column for X (0-2): ").split())
        if board[x_row][x_col] != EMPTY:
            print("Invalid move! Try again.")
            continue
        board[x_row][x_col] = PLAYER_X

        if check_winner(board):
            print_board(board)
            print("Player X wins!")
            break

        if is_full(board):
            print("It's a draw!")
            break

        
        o_row, o_col = find_best_move(board)
        board[o_row][o_col] = PLAYER_O

        if check_winner(board):
            print_board(board)
            print("Player O wins!")
            break

if __name__ == "__main__":
    play_game()