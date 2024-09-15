using Pkg; Pkg.activate(@__DIR__); Pkg.develop(path = joinpath(@__DIR__, ".."))
using Documenter
using PinnZoo

makedocs(sitename="PinnZoo Documentation")