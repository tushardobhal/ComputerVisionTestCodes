#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sndfile.h>
#include <fftw3.h>
#include <string.h>
#include <math.h>
#include <vector>

using namespace std;

#define FFT_BUF_SIZE  880
#define HOP_SIZE (FFT_BUF_SIZE/2)
#define WORD_SIZE 13
#define PI 3.14159265358979323846264338327

class MFCC
{

  double hamming[FFT_BUF_SIZE];
/*
 * Compute the center frequency (fc) of the specified filter band (l) (Eq. 4)
 * This where the mel-frequency scaling occurs. Filters are specified so that their
 * center frequencies are equally spaced on the mel scale
 * Used for internal computation only - not the be called directly
 */
  double GetCenterFrequency(unsigned int filterBand)
  {
        double centerFrequency = 0.0f;
        double exponent;

        if(filterBand == 0)
        {
                centerFrequency = 0;
        }
        else if(filterBand >= 1 && filterBand <= 14)
        {
                centerFrequency = (200.0f * filterBand) / 3.0f;
        }
        else
        {
                exponent = filterBand - 14.0f;
                centerFrequency = pow(1.0711703, exponent);
                centerFrequency *= 1073.4;
        }
       
        return centerFrequency;
  }

/*
 * Compute the band-dependent magnitude factor for the given filter band (Eq. 3)
 * Used for internal computation only - not the be called directly
 */
  double GetMagnitudeFactor(unsigned int filterBand)
  {
        double magnitudeFactor = 0.0f;
       
        if(filterBand >= 1 && filterBand <= 14)
        {
                magnitudeFactor = 0.015;
        }
        else if(filterBand >= 15 && filterBand <= 48)
        {
                magnitudeFactor = 2.0f / (GetCenterFrequency(filterBand + 1) - GetCenterFrequency(filterBand -1));
        }

        return magnitudeFactor;
  }



/*
 * Compute the filter parameter for the specified frequency and filter bands (Eq. 2)
 * Used for internal computation only - not the be called directly
 */
  double GetFilterParameter(unsigned int samplingRate, unsigned int binSize, unsigned int frequencyBand, unsigned int filterBand)
  {
        double filterParameter = 0.0f;

        double boundary = (frequencyBand * samplingRate) / binSize;             // k * Fs / N
        double prevCenterFrequency = GetCenterFrequency(filterBand - 1);                // fc(l - 1) etc.
        double thisCenterFrequency = GetCenterFrequency(filterBand);
        double nextCenterFrequency = GetCenterFrequency(filterBand + 1);

        if(boundary >= 0 && boundary < prevCenterFrequency)
        {
                filterParameter = 0.0f;
        }
        else if(boundary >= prevCenterFrequency && boundary < thisCenterFrequency)
        {
                filterParameter = (boundary - prevCenterFrequency) / (thisCenterFrequency - prevCenterFrequency);
                filterParameter *= GetMagnitudeFactor(filterBand);
        }
        else if(boundary >= thisCenterFrequency && boundary < nextCenterFrequency)
        {
                filterParameter = (boundary - nextCenterFrequency) / (thisCenterFrequency - nextCenterFrequency);
                filterParameter *= GetMagnitudeFactor(filterBand);
        }
        else if(boundary >= nextCenterFrequency && boundary < samplingRate)
        {
                filterParameter = 0.0f;
        }

        return filterParameter;
  }


/*
 * Computes the Normalization Factor (Equation 6)
 * Used for internal computation only - not to be called directly
 */
  double NormalizationFactor(int NumFilters, int m)
  {
        double normalizationFactor = 0.0f;

        if(m == 0)
        {
                normalizationFactor = sqrt(1.0f / NumFilters);
        }
        else
        {
                normalizationFactor = sqrt(2.0f / NumFilters);
        }
       
        return normalizationFactor;
  }

  double GetCoefficient(double* spectralData, unsigned int samplingRate, unsigned int NumFilters, unsigned int binSize, unsigned int m)
  {
        double result = 0.0f;
        double outerSum = 0.0f;
        double innerSum = 0.0f;
        unsigned int k, l;

        // 0 <= m < L
        if(m >= NumFilters)
        {
                // This represents an error condition - the specified coefficient is greater than or equal to the number of filters. The behavior in this case is undefined.
                return 0.0f;
        }

        result = NormalizationFactor(NumFilters, m);

       
        for(l = 1; l <= NumFilters; l++)
        {
                // Compute inner sum
                innerSum = 0.0f;
                for(k = 0; k < binSize - 1; k++)
                {
                        innerSum += fabs(spectralData[k] * GetFilterParameter(samplingRate, binSize, k, l));
                }

                if(innerSum > 0.0f)
                {
                        innerSum = log(innerSum); // The log of 0 is undefined, so don't use it
                }

                innerSum = innerSum * cos(((m * PI) / NumFilters) * (l - 0.5f));

                outerSum += innerSum;
        }

        result *= outerSum;

        return result;
  }

  public:

  void initWindow(void){
    for (int i = 0; i < FFT_BUF_SIZE; i++){
        hamming[i] = 0.54 - 0.46*cos((2*M_PI*i)/(FFT_BUF_SIZE - 1));
    }
  }

  vector<vector<double> > calcMFCC(double ** wav_p, int blocks_read, int mfcc_order, int sr)
  {
    //COMPUTE THE FFTs
    fftw_complex *out;
    double *in;
    double *wav = *wav_p;
    vector<vector<double> > mfccResult;
    fftw_plan p;

    double spectrum[FFT_BUF_SIZE];
    double curCoeff;

    in = (double*) fftw_malloc(sizeof(double)*FFT_BUF_SIZE);
    out = (fftw_complex*) fftw_malloc(sizeof(fftw_complex)*(FFT_BUF_SIZE/2+1));
    p = fftw_plan_dft_r2c_1d(FFT_BUF_SIZE, in, out, FFTW_MEASURE);

    int pos;
    int win_num = 0;
    for (pos = 0; pos < blocks_read - FFT_BUF_SIZE; pos += HOP_SIZE){
        vector <double> curMFCC;
        win_num++;
        int i;
        for (i = 0; i < FFT_BUF_SIZE; i++){
            in[i] = wav[pos+i]*hamming[i];
        }
        fftw_execute(p);
        for (i = 0; i < FFT_BUF_SIZE/2+1; i++){
            spectrum[i] = out[i][0]/FFT_BUF_SIZE;
        }
        int coeff;
        for(coeff = 0; coeff < mfcc_order; coeff++) {
            curCoeff = GetCoefficient(spectrum, sr, 48, 128, coeff);
            curMFCC.push_back(curCoeff);
        }
        mfccResult.push_back(curMFCC);
        printf("\r%d %% complete", (int)((pos*100/(blocks_read- FFT_BUF_SIZE))+1) );
        fflush(stdout);
    }
    cout<<"\n";
    return mfccResult;
  }


  void writeFile(vector<vector<double> > mfccCoeff, char* dictFname)
  {
    FILE * outfileraw = fopen(dictFname, "w");
    for(int i = 0; i < mfccCoeff.size(); i++)
    {   
        for(int j = 0; j < mfccCoeff[i].size(); j++)
        {
            fprintf(outfileraw, "%f ", mfccCoeff[i][j]);                    
        }
        fprintf(outfileraw, "\n\n");
    }
  }

 
  vector<vector<double> > compMFCC(char* infilename, int size)
  {
    initWindow();
    double *wav;
    vector<vector<double> > mfccResult;
    SNDFILE *infile;
    SF_INFO info;
    int readcount;
    if(size != 0)
        infilename[size] = '\0';    

    if(!(infile = sf_open (infilename, SFM_READ, &info)))
    {   
        printf ("Not able to open input file %s.\n", infilename);
        puts (sf_strerror (NULL));
        exit(0);
    }
    if(info.channels != 1)
    {
         printf("Only tested on single channel audio.\n");
         exit(0);
    }
    wav = (double *) malloc(info.frames*sizeof(double));
    readcount = sf_read_double(infile, wav, info.frames);
    sf_close (infile) ;
    printf("Computing MFCC for %s\n",infilename); 
    
    for(int i=1;i<info.frames;i++)
        wav[i] = wav[i] - (0.97*wav[i-1]);
    
    mfccResult = calcMFCC(&wav, info.frames, WORD_SIZE, info.samplerate);
    free(wav);

    return mfccResult;
  }
};
