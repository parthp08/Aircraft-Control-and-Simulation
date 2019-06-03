using DelimitedFiles
export module_functions

function module_functions(f)
    """
    returns all functions in the module in text file
    """
    modulename = f
    A = names(f)
    open("$(f)_functions.txt", "w") do io
        writedlm(io, [A], '\n')
    end
end

###### Example
# using ControlSystems
# module_functions(ControlSystems)
