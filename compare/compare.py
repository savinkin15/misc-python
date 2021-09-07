import pandas as pd
import numpy as np

columns = "B:K"
order_by = ['Date', 'HomeTeam', 'AwayTeam']
# order_by = ['Date', 'HomeTeam', 'AwayTeam']
order_by_values = [True, True, True]

# Tobe removed
file_1 = 'BCA.xlsx'
file_2 = 'IAM.xlsx'

column_type = {
    'Tottal Home goals': int,
    'Tottal Away Goals ': int,
    'Half Time Home Goals': int,
    'Half Time Away Goals': int,
    'HOME TRIES ': float,
    'AWAY TRIES ': float
}

def print_dif(data1, data2, index):
    bca_str_list = []
    iam_str_list = []
    for item in column_type.keys():
        if (data1[item] != data2[item]):
            bca_str_list.append(f"{  column_type[item](data1[item]) } {item}")
            iam_str_list.append(f"{ column_type[item](data2[item]) } {item}")
    bca_str = ' AND '.join(bca_str_list)
    iam_str = ' AND '.join(iam_str_list)
    print (f'BCA has listed { bca_str } for the {index[0].strftime("%m/%d/%Y")} {index[1]}/{index[2]} game.   And "IAM has {iam_str} Listed on that game')

def print_add_data(add_data, index, tbl_idx=0):
    str_list = []
    sheets =["BCA", "IAM"]
    for item in column_type.keys():
        str_list.append(f"{  column_type[item](add_data[item]) } {item}")
    add_str = ' AND '.join(str_list)
    print (f'{sheets[tbl_idx]} has listed { add_str } for the {index[0]} {index[1]}/{index[2]} game.   And {sheets[1-tbl_idx]} does not have that game')


def compare(file_1, columns_1, file_2, columns_2, order_by, order_by_values):

    df1=pd.read_excel(file_1, index_col=0, na_values=['NA'], usecols=columns_1, engine='openpyxl')
    df2=pd.read_excel(file_2, index_col=0, na_values=['NA'], usecols=columns_2, engine='openpyxl')

    df1.reset_index(inplace=True)
    df1 = df1.set_index(order_by)
    len_origin = len(df1)
    df1 = df1.dropna()
    null_len_1 = len_origin - len(df1)

    df2.reset_index(inplace=True)
    df2 = df2.set_index(order_by)
    len_origin = len(df2)
    df2 = df2.dropna()
    null_len_2 = len_origin - len(df2)

    # print blanks
    if not null_len_1 == 0:
        print(f"BCA dataset has {null_len_1} blank rows that are absent from the IAM document")
    if not null_len_2 == 0:
        print(f"IAM dataset has {null_len_2} blank rows that are absent from the BCA document")

    df1 = df1.astype('float64')
    df2 = df2.astype('float64')
    
    for data in df1.index:
        df1_data = df1.loc[data]
        df2_data = df2.loc[data]
        if not df1_data.equals(df2_data):
            print_dif(df1_data, df2_data, data)

    for data in df1.index.difference(df2.index):
        df1_data = df1.loc[data]
        print_add_data(df1_data, data, 0)

    for data in df2.index.difference(df1.index):
        df1_data = df2.loc[data]
        print_add_data(df2_data, data, 1)

# df1=pd.read_excel(file_1, index_col=0, na_values=['NA'], usecols=columns, engine='openpyxl')
# df1.reset_index(inplace=True)
# df1 = df1.set_index(order_by)

# len_origin = len(df1)
# df1 = df1.dropna()
# null_len_1 = len_origin - len(df1)

# df2=pd.read_excel(file_2, index_col=0, na_values=['NA'], usecols=columns, engine='openpyxl')
# df2.reset_index(inplace=True)
# df2 = df2.set_index(order_by)

# len_origin = len(df1)
# df2 = df2.dropna()
# null_len_2 = len_origin - len(df2)

# df1 = df1.astype('float64')
# df2 = df2.astype('float64')

# for data in df1.index:
#     df1_data = df1.loc[data]
#     df2_data = df2.loc[data]
#     if not df1_data.equals(df2_data):
#         print_dif(df1_data, df2_data, data)

# for data in df1.index.difference(df2.index):
#     df1_data = df1.loc[data]
#     print_add_data(df1_data, data, 0)

# for data in df2.index.difference(df1.index):
#     df1_data = df2.loc[data]
#     print_add_data(df2_data, data, 1)


# end of to be removed
compare('BCA.xlsx', columns, 'IAM.xlsx', columns, order_by, order_by_values)