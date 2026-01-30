import numpy as np

def rot(theeta = [0, 0, 0, 0, 0, 0]):
    final = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])
    theeta = list(map(lambda i: (i / 180) * np.pi, theeta))

    projection_matrices = [
        [[1, 0, 0], [0, 0, 1], [0, -1, 0]], #1
        [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], #2
        [[1, 0, 0], [0, 0, 1], [0, -1, 0]], #3
        [[1, 0, 0], [0, 0, -1], [0, 1, 0]], #4
        [[1, 0, 0], [0, 0, 1], [0, -1, 0]], #5
        [[1, 0, 0], [0, 1, 0], [0, 0, 1]]   #6
    ]

    projection_matrices = [np.array(matrix) for matrix in projection_matrices]

    for i, theta in enumerate(theeta):
        rotation_matrix = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

        projection_matrix = projection_matrices[i]
        rot = np.dot(rotation_matrix, projection_matrix)
        final = np.dot(final,rot)
        #print(final)
    return final

def main():
        final = rot()
        print(f"{np.matrix(final)}\n")

if __name__ == "__main__":
    main()
