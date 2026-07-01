#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <iomanip>
#include <chrono>
#include <omp.h>
#include <immintrin.h>

using namespace std;

// Vector constants for AVX2 fast cosine
const __m256 _ps_one_over_twopi = _mm256_set1_ps(0.159154943f);
const __m256 _ps_twopi = _mm256_set1_ps(6.283185307f);
const __m256 _ps_abs_mask = _mm256_castsi256_ps(_mm256_set1_epi32(0x7FFFFFFF));
const __m256 _ps_c2 = _mm256_set1_ps(-0.4999999963f);
const __m256 _ps_c4 = _mm256_set1_ps(0.0416666418f);
const __m256 _ps_c6 = _mm256_set1_ps(-0.0013888397f);
const __m256 _ps_c8 = _mm256_set1_ps(0.0000247609f);
const __m256 _ps_c10 = _mm256_set1_ps(-0.0000002605f);
const __m256 _ps_one = _mm256_set1_ps(1.0f);

inline __m256 fast_cos_ps(__m256 x) {
    // 1. Range reduction: x_red = x - round(x / 2pi) * 2pi
    __m256 z = _mm256_round_ps(_mm256_mul_ps(x, _ps_one_over_twopi), _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
    __m256 x_red = _mm256_fnmadd_ps(z, _ps_twopi, x);
    
    // 2. Take absolute value
    __m256 x_abs = _mm256_and_ps(x_red, _ps_abs_mask);
    
    // 3. Polynomial approximation
    __m256 y2 = _mm256_mul_ps(x_abs, x_abs);
    __m256 poly = _mm256_fmadd_ps(y2, _ps_c10, _ps_c8);
    poly = _mm256_fmadd_ps(y2, poly, _ps_c6);
    poly = _mm256_fmadd_ps(y2, poly, _ps_c4);
    poly = _mm256_fmadd_ps(y2, poly, _ps_c2);
    poly = _mm256_fmadd_ps(y2, poly, _ps_one);
    
    return poly;
}

int main(int argc, char* argv[]) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <input_data_file> <output_crg_file>" << endl;
        return 1;
    }
    
    string input_path = argv[1];
    string output_path = argv[2];
    
    ifstream infile(input_path);
    if (!infile.is_open()) {
        cerr << "Error opening input file: " << input_path << endl;
        return 1;
    }
    
    int Nu, Nv, N_waves;
    double u_length, du, v_right, v_left, dv;
    if (!(infile >> Nu >> Nv >> N_waves >> u_length >> du >> v_right >> v_left >> dv)) {
        cerr << "Error reading header from " << input_path << endl;
        return 1;
    }
    
    vector<double> phi_grid(Nu);
    vector<double> x_ref(Nu);
    vector<double> y_ref(Nu);
    vector<double> slope_grid(Nu);
    vector<double> banking_grid(Nu);
    for (int i = 0; i < Nu; ++i) {
        if (!(infile >> phi_grid[i] >> x_ref[i] >> y_ref[i] >> slope_grid[i] >> banking_grid[i])) {
            cerr << "Error reading reference line at index " << i << endl;
            return 1;
        }
    }
    
    vector<float> amps(N_waves);
    vector<float> kx(N_waves);
    vector<float> ky(N_waves);
    vector<float> phis(N_waves);
    for (int w = 0; w < N_waves; ++w) {
        if (!(infile >> amps[w] >> kx[w] >> ky[w] >> phis[w])) {
            cerr << "Error reading wave at index " << w << endl;
            return 1;
        }
    }
    infile.close();

    vector<double> z_ref(Nu, 0.0);
    for (int i = 1; i < Nu; ++i) {
        z_ref[i] = z_ref[i - 1] + du * slope_grid[i - 1];
    }
    
    cout << "Read inputs successfully. Nu=" << Nu << ", Nv=" << Nv << ", N_waves=" << N_waves << endl;
    
    vector<double> v_grid(Nv);
    for (int j = 0; j < Nv; ++j) {
        v_grid[j] = v_right + j * dv;
    }
    
    vector<float> grid_x(Nu * Nv, 0.0f);
    vector<float> grid_y(Nu * Nv, 0.0f);
    vector<float> grid_z(Nu * Nv, 0.0f);
    
    // Pad waves to multiple of 8
    int N_waves_padded = ((N_waves + 7) / 8) * 8;
    vector<float> amps_pad(N_waves_padded, 0.0f);
    vector<float> kx_pad(N_waves_padded, 0.0f);
    vector<float> ky_pad(N_waves_padded, 0.0f);
    vector<float> phis_pad(N_waves_padded, 0.0f);
    for (int w = 0; w < N_waves; ++w) {
        amps_pad[w] = amps[w];
        kx_pad[w] = kx[w];
        ky_pad[w] = ky[w];
        phis_pad[w] = phis[w];
    }
    
    // Parallel computation of z_grid and formatting of row strings
    vector<string> row_strings(Nu);
    
    auto t0 = chrono::high_resolution_clock::now();
    
    #pragma omp parallel
    {
        int lines_per_row = (3 + Nv + 7) / 8;
        string local_buffer;
        local_buffer.reserve(lines_per_row * 85);
        
        #pragma omp for schedule(dynamic, 64)
        for (int i = 0; i < Nu; ++i) {
            double phi = phi_grid[i];
            double xr = x_ref[i];
            double yr = y_ref[i];
            double slope = slope_grid[i];
            double banking = banking_grid[i];
            double sin_phi = sin(phi);
            double cos_phi = cos(phi);
            double cos_bank = sqrt(1.0 - banking * banking);
            double sin_bank = banking;
            
            local_buffer.clear();
            
            char val_buf[32];
            int col_in_line = 0;
            
            // 1. Format phi
            sprintf(val_buf, "%10.7f", phi);
            local_buffer.append(val_buf);
            col_in_line++;
            
            // 2. Format slope
            sprintf(val_buf, "%10.7f", slope);
            local_buffer.append(val_buf);
            col_in_line++;
            if (col_in_line == 8) { local_buffer.append("\n"); col_in_line = 0; }
            
            // 3. Format banking
            sprintf(val_buf, "%10.7f", banking);
            local_buffer.append(val_buf);
            col_in_line++;
            if (col_in_line == 8) { local_buffer.append("\n"); col_in_line = 0; }
            
            // 4. Format Nv elevations using AVX2-accelerated math
            for (int j = 0; j < Nv; ++j) {
                double v = v_grid[j];
                // Flat projection coordinates for evaluating roughness waves (matches OpenCRG C-API projection)
                double x_crg = xr - v * sin_phi;
                double y_crg = yr + v * cos_phi;
                
                // Physical projection coordinates for the 3D OBJ mesh (retains width scaling under banking)
                double x_mesh = xr - v * cos_bank * sin_phi;
                double y_mesh = yr + v * cos_bank * cos_phi;
                
                double z_bank = v * sin_bank;
                
                __m256 sum_vec = _mm256_setzero_ps();
                __m256 x_vec = _mm256_set1_ps((float)x_crg);
                __m256 y_vec = _mm256_set1_ps((float)y_crg);
                
                for (int w = 0; w < N_waves_padded; w += 8) {
                    __m256 kx_v = _mm256_loadu_ps(&kx_pad[w]);
                    __m256 ky_v = _mm256_loadu_ps(&ky_pad[w]);
                    __m256 phis_v = _mm256_loadu_ps(&phis_pad[w]);
                    __m256 amps_v = _mm256_loadu_ps(&amps_pad[w]);
                    
                    __m256 arg = _mm256_fmadd_ps(kx_v, x_vec, phis_v);
                    arg = _mm256_fmadd_ps(ky_v, y_vec, arg);
                    
                    __m256 cos_val = fast_cos_ps(arg);
                    sum_vec = _mm256_fmadd_ps(amps_v, cos_val, sum_vec);
                }
                
                __m128 low = _mm256_castps256_ps128(sum_vec);
                __m128 high = _mm256_extractf128_ps(sum_vec, 1);
                __m128 sum128 = _mm_add_ps(low, high);
                sum128 = _mm_hadd_ps(sum128, sum128);
                sum128 = _mm_hadd_ps(sum128, sum128);
                float total_z = _mm_cvtss_f32(sum128);
                
                float final_z = (float)(z_ref[i] + z_bank + total_z);
                
                grid_x[i * Nv + j] = (float)x_mesh;
                grid_y[i * Nv + j] = (float)y_mesh;
                grid_z[i * Nv + j] = final_z;
                
                sprintf(val_buf, "%10.7f", (double)total_z);
                local_buffer.append(val_buf);
                
                col_in_line++;
                if (col_in_line == 8) {
                    local_buffer.append("\n");
                    col_in_line = 0;
                }
            }
            if (col_in_line > 0) {
                local_buffer.append("\n");
            }
            
            row_strings[i] = local_buffer;
        }
    }
    
    auto t1 = chrono::high_resolution_clock::now();
    double elapsed_comp = chrono::duration<double>(t1 - t0).count();
    cout << "Computation and string formatting took " << elapsed_comp << " seconds." << endl;
    
    // Write to file sequentially
    ofstream outfile(output_path);
    if (!outfile.is_open()) {
        cerr << "Error opening output file for writing: " << output_path << endl;
        return 1;
    }
    
    outfile << "$ROAD_CRG\n";
    outfile << "reference_line_start_u    =  0.0\n";
    outfile << "reference_line_end_u      =  " << fixed << setprecision(3) << u_length << "\n";
    outfile << "reference_line_increment  =  " << du << "\n";
    outfile << "long_section_v_right      =  " << v_right << "\n";
    outfile << "long_section_v_left       =  " << v_left << "\n";
    outfile << "long_section_v_increment  =  " << dv << "\n";
    outfile << "reference_line_start_x    =  " << fixed << setprecision(6) << x_ref[0] << "\n";
    outfile << "reference_line_start_y    =  " << y_ref[0] << "\n";
    outfile << "reference_line_start_phi  =  " << phi_grid[0] << "\n";
    outfile << "$\n";
    outfile << "$ROAD_CRG_OPTS\n";
    outfile << "refline_continuation = 1.0\n";
    outfile << "$\n";
    outfile << "$KD_DEFINITION\n";
    outfile << "#:LRFI\n";
    outfile << "U:reference line u,m,0.000," << du << "\n";
    outfile << "D:reference line phi,rad\n";
    outfile << "D:reference line slope,m/m\n";
    outfile << "D:reference line banking,m/m\n";
    for (int col = 0; col < Nv; ++col) {
        outfile << "D:long section " << (col + 1) << ",m\n";
    }
    outfile << "$\n";
    outfile << "$$$$\n";
    
    for (int i = 0; i < Nu; ++i) {
        outfile << row_strings[i];
    }
    outfile.close();
    
    if (argc > 3) {
        string obj_path = argv[3];
        if (!obj_path.empty() && obj_path != "\"\"") {
            cout << "Writing OBJ mesh to " << obj_path << "..." << endl;
            FILE* f_obj = fopen(obj_path.c_str(), "w");
            if (!f_obj) {
                cerr << "Error opening OBJ file for writing: " << obj_path << endl;
            } else {
                // Set a large 1MB buffer for fast I/O
                vector<char> buffer(1024 * 1024);
                setvbuf(f_obj, buffer.data(), _IOFBF, buffer.size());
                
                fprintf(f_obj, "# Road Surface OBJ Mesh\n");
                // Write vertices
                for (int idx = 0; idx < Nu * Nv; ++idx) {
                    fprintf(f_obj, "v %.4f %.4f %.4f\n", grid_x[idx], grid_y[idx], grid_z[idx]);
                }
                // Write texture coordinates (UVs)
                double road_length = u_length;
                double road_width = v_left - v_right;
                double v_repeats = 0.5 * road_length / (road_width > 0.0 ? road_width : 8.0);
                for (int i = 0; i < Nu; ++i) {
                    for (int j = 0; j < Nv; ++j) {
                        double u_coord = (double)j / (Nv - 1);
                        double v_coord = ((double)i / (Nu - 1)) * v_repeats;
                        fprintf(f_obj, "vt %.4f %.4f\n", u_coord, v_coord);
                    }
                }
                // Write faces linking vertices and texture coordinates
                for (int i = 0; i < Nu - 1; ++i) {
                    for (int j = 0; j < Nv - 1; ++j) {
                        int v1 = i * Nv + j + 1;
                        int v2 = i * Nv + (j + 1) + 1;
                        int v3 = (i + 1) * Nv + (j + 1) + 1;
                        int v4 = (i + 1) * Nv + j + 1;
                        fprintf(f_obj, "f %d/%d %d/%d %d/%d %d/%d\n", v1, v1, v4, v4, v3, v3, v2, v2);
                    }
                }
                fclose(f_obj);
                cout << "OBJ mesh written successfully." << endl;
            }
        }
    }
    
    auto t2 = chrono::high_resolution_clock::now();
    double elapsed_write = chrono::duration<double>(t2 - t1).count();
    cout << "Writing to CRG file took " << elapsed_write << " seconds." << endl;
    cout << "Total C++ generator time: " << elapsed_comp + elapsed_write << " seconds." << endl;
    
    return 0;
}
