export print_matrix

function print_matrix(matrix_::Array)
    println("[")
    for i = 1:size(matrix_)[1]
        println(matrix_[i,:])
    end
    println("]")
end
