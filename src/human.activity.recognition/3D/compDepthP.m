function [] = compDepthP()
    for i=1:20
        for j=1:10
            for k=1:3
                i
                if(i<10)
                    if(j<10)
                        name = sprintf('MSR-Action3D/a0%d/a0%d_s0%d_e0%d_sdepth.mat', i, i, j, k);
                    else
                        name = sprintf('MSR-Action3D/a0%d/a0%d_s%d_e0%d_sdepth.mat', i, i, j, k);
                    end
                else
                    if(j<10)
                        name = sprintf('MSR-Action3D/a%d/a%d_s0%d_e0%d_sdepth.mat', i, i, j, k);
                    else
                        name = sprintf('MSR-Action3D/a%d/a%d_s%d_e0%d_sdepth.mat', i, i, j, k);
                    end
                end
                if(exist(name, 'file'))
                    load(name);
                    [F,S,T] = depth_projection(depth);
                    imwrite(F, sprintf('MSR-Action3D/images/%d/F_a%d_s%d_e0%d.jpg', i, i, j, k));
                    imwrite(S, sprintf('MSR-Action3D/images/%d/S_a%d_s%d_e0%d.jpg', i, i, j, k));
                    imwrite(T, sprintf('MSR-Action3D/images/%d/T_a%d_s%d_e0%d.jpg', i, i, j, k));
                end
            end
        end
    end
end