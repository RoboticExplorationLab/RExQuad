using RecipesBase
@userplot PlotStates 

@recipe function f(ps::PlotStates; inds=1:length(ps.args[end][1]))
    Xvec = ps.args[end]
    if length(ps.args) == 1
        times = 1:length(Xvecs)
    else
        times = ps.args[1]
    end
    Xmat = reduce(hcat,Xvec)[inds,:]'
    (times,Xmat)
end