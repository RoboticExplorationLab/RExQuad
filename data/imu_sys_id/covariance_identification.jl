using Distributions
using StatsPlots
using Plots; gr()
using CSV
using DataFrames
using Plots.PlotMeasures

# %%
df = DataFrame(CSV.File("test.csv"))
inds = ((abs.(diff(df.acc_x)) .< 0.1) .& (abs.(diff(df.acc_y)) .< 0.1) .& (abs.(diff(df.acc_z)) .< 0.1) .&
        (abs.(diff(df.gyr_x)) .< 0.5) .& (abs.(diff(df.gyr_y)) .< 0.5) .& (abs.(diff(df.gyr_z)) .< 0.5));
diff_df = DataFrame(diff(Matrix(df), dims=1)[inds, :], names(df));

# %%
title = plot(title = "IMU Acceleration Brownian Noise", grid=false,
             showaxis=false, ticks=false, right_margin=5mm)

p1 = histogram(diff_df.acc_x, bins=:scott,
               xlims=[-0.1, 0.1], title="X Acceleration")
dist1 = fit(Normal{Float64}, diff_df.acc_x)
annotate!(p1, .075, 1500, "σ = $(round(params(dist1)[2]; digits=3))", :black)
p1 = plot!(twinx(), [-.1:.001:.1;], pdf(dist1, [-.1:.001:.1;]), xticks=false,
           linecolor=2, linewidth=3)

p2 = histogram(diff_df.acc_y, bins=:scott,
               xlims=[-.1, .1], title="Y Acceleration")
dist2 = fit(Normal{Float64}, diff_df.acc_y)
annotate!(p2, .075, 1500, "σ = $(round(params(dist2)[2]; digits=3))", :black)
p2 = plot!(twinx(), [-.1:.001:.1;], pdf(dist2, [-.1:.001:.1;]), xticks=false,
           linecolor=2, linewidth=3)

p3 = histogram(diff_df.acc_z, bins=:scott,
               xlims=[-.1, .1], title="Z Acceleration")
dist3 = fit(Normal{Float64}, diff_df.acc_z)
annotate!(p3, .075, 1500, "σ = $(round(params(dist3)[2]; digits=3))", :black)
p3 = plot!(twinx(), [-.1:.001:.1;], pdf(dist3, [-.1:.001:.1;]), xticks=false,
           linecolor=2, linewidth=3)

p4 = plot(title, p1, p2, p3, size=(550, 550), legend=false, layout=@layout([A{0.01h}; B; C; D]))
savefig(p4, "AccelerationBrownianNoise.png")
p4

# %%
title = plot(title = "IMU Angular Rate Brownian Noise", grid=false,
             showaxis=false, ticks=false, right_margin=5mm)

p1 = histogram(diff_df.gyr_x, bins=:scott, xlims=[-0.5, 0.5], title="X Angular Rate")
dist1 = fit(Normal{Float64}, diff_df.gyr_x)
annotate!(p1, .375, 1500, "σ = $(round(params(dist1)[2]; digits=3))", :black)
p1 = plot!(twinx(), [-.5:.01:.5;], pdf(dist, [-.5:.01:.5;]), xticks=false,
           linecolor=2, linewidth=3)

p2 = histogram(diff_df.gyr_y, bins=:scott, xlims=[-0.5, 0.5], title="Y Angular Rate")
dist2 = fit(Normal{Float64}, diff_df.gyr_y)
annotate!(p2, .375, 1500, "σ = $(round(params(dist2)[2]; digits=3))", :black)
p2 = plot!(twinx(), [-.5:.01:.5;], pdf(dist, [-.5:.01:.5;]), xticks=false,
           linecolor=2, linewidth=3)

p3 = histogram(diff_df.gyr_z, bins=:scott, xlims=[-0.5, 0.5], title="Z Angular Rate")
dist3 = fit(Normal{Float64}, diff_df.gyr_z)
annotate!(p3, .375, 1500, "σ = $(round(params(dist3)[2]; digits=3))", :black)
p3 = plot!(twinx(), [-.5:.01:.5;], pdf(dist, [-.5:.01:.5;]), xticks=false,
           linecolor=2, linewidth=3)

p4 = plot(title, p1, p2, p3, size=(550, 550), legend=false, layout=@layout([A{0.01h}; B; C; D]))
savefig(p4, "AngularBrownianNoise.png")
p4

# %%
dist = fit(MvNormal, Matrix(diff_df[!, 1:6])')

# %%
using DelimitedFiles
writedlm( "mean.csv",  mean(dist), ',')
writedlm( "covariance.csv",  cov(dist), ',')
